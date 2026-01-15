use std::fs::{File, OpenOptions};
use std::io::Write;
use std::os::unix::io::AsRawFd;
use std::thread;
use std::time::Duration;
use std::ptr;

// GPIO pins for I2C bitbanging
const GPIO_SDA: u32 = 10;
const GPIO_SCL: u32 = 11;
const GPIO_INT: u32 = 27; // Touch interrupt (active low)

// I2C parameters
const I2C_ADDR: u8 = 0x15;
const TOUCH_POINTS: usize = 2;
const SCREEN_WIDTH: i32 = 480;
const SCREEN_HEIGHT: i32 = 480;

// FT5x06 registers
const REG_TD_STATUS: u8 = 0x02;
const REG_TOUCH_START: u8 = 0x03;

// BCM2835 GPIO registers (directly via gpiomem)
const GPIO_BASE_OFFSET: usize = 0;
const GPFSEL0: usize = 0x00 / 4;
const GPSET0: usize = 0x1C / 4;
const GPCLR0: usize = 0x28 / 4;
const GPLEV0: usize = 0x34 / 4;
const GPPUD: usize = 0x94 / 4;
const GPPUDCLK0: usize = 0x98 / 4;

// GPIO function select values
const GPIO_FSEL_INPUT: u32 = 0b000;
const GPIO_FSEL_OUTPUT: u32 = 0b001;

const UINPUT_PATH: &str = "/dev/uinput";

// Event types
const EV_SYN: u16 = 0x00;
const EV_KEY: u16 = 0x01;
const EV_ABS: u16 = 0x03;

// Sync events
const SYN_REPORT: u16 = 0x00;

// Key codes
const BTN_TOUCH: u16 = 0x14a;

// Absolute axes
const ABS_X: u16 = 0x00;
const ABS_Y: u16 = 0x01;
const ABS_MT_SLOT: u16 = 0x2f;
const ABS_MT_TRACKING_ID: u16 = 0x39;
const ABS_MT_POSITION_X: u16 = 0x35;
const ABS_MT_POSITION_Y: u16 = 0x36;

#[repr(C)]
struct InputId {
    bustype: u16,
    vendor: u16,
    product: u16,
    version: u16,
}

#[repr(C)]
struct AbsInfo {
    value: i32,
    minimum: i32,
    maximum: i32,
    fuzz: i32,
    flat: i32,
    resolution: i32,
}

#[repr(C)]
struct UinputAbsSetup {
    code: u16,
    absinfo: AbsInfo,
}

#[repr(C)]
struct UinputSetup {
    id: InputId,
    name: [u8; 80],
    ff_effects_max: u32,
}

#[repr(C)]
struct InputEvent {
    tv_sec: libc::time_t,
    tv_usec: libc::suseconds_t,
    type_: u16,
    code: u16,
    value: i32,
}

nix::ioctl_write_int!(ui_set_evbit, b'U', 100);
nix::ioctl_write_int!(ui_set_keybit, b'U', 101);
nix::ioctl_write_int!(ui_set_absbit, b'U', 103);
nix::ioctl_write_ptr!(ui_abs_setup, b'U', 4, UinputAbsSetup);
nix::ioctl_write_ptr!(ui_dev_setup, b'U', 3, UinputSetup);
nix::ioctl_none!(ui_dev_create, b'U', 1);
nix::ioctl_none!(ui_dev_destroy, b'U', 2);

struct TouchPoint {
    id: i32,
    x: i32,
    y: i32,
    active: bool,
}

/// GPIO memory-mapped I/O for bitbanging
struct GpioMem {
    map: *mut u32,
    _file: File,
}

impl GpioMem {
    fn new() -> std::io::Result<Self> {
        let file = OpenOptions::new()
            .read(true)
            .write(true)
            .open("/dev/gpiomem")?;

        let map = unsafe {
            libc::mmap(
                ptr::null_mut(),
                4096,
                libc::PROT_READ | libc::PROT_WRITE,
                libc::MAP_SHARED,
                file.as_raw_fd(),
                0,
            )
        };

        if map == libc::MAP_FAILED {
            return Err(std::io::Error::last_os_error());
        }

        Ok(Self {
            map: map as *mut u32,
            _file: file,
        })
    }

    fn set_pin_mode(&self, pin: u32, mode: u32) {
        let reg = (pin / 10) as usize;
        let shift = (pin % 10) * 3;
        unsafe {
            let ptr = self.map.add(GPFSEL0 + reg);
            let val = ptr::read_volatile(ptr);
            let val = (val & !(0b111 << shift)) | (mode << shift);
            ptr::write_volatile(ptr, val);
        }
    }

    fn set_pin_high(&self, pin: u32) {
        unsafe {
            let ptr = self.map.add(GPSET0);
            ptr::write_volatile(ptr, 1 << pin);
        }
    }

    fn set_pin_low(&self, pin: u32) {
        unsafe {
            let ptr = self.map.add(GPCLR0);
            ptr::write_volatile(ptr, 1 << pin);
        }
    }

    fn read_pin(&self, pin: u32) -> bool {
        unsafe {
            let ptr = self.map.add(GPLEV0);
            (ptr::read_volatile(ptr) & (1 << pin)) != 0
        }
    }

    fn set_pullup(&self, pin: u32) {
        unsafe {
            // Set pull-up mode
            let pud_ptr = self.map.add(GPPUD);
            ptr::write_volatile(pud_ptr, 2); // 2 = pull-up
            self.delay_us(5);

            let clk_ptr = self.map.add(GPPUDCLK0);
            ptr::write_volatile(clk_ptr, 1 << pin);
            self.delay_us(5);

            ptr::write_volatile(pud_ptr, 0);
            ptr::write_volatile(clk_ptr, 0);
        }
    }

    fn delay_us(&self, us: u32) {
        std::thread::sleep(Duration::from_micros(us as u64));
    }
}

impl Drop for GpioMem {
    fn drop(&mut self) {
        unsafe {
            libc::munmap(self.map as *mut libc::c_void, 4096);
        }
    }
}

/// Bitbanged I2C implementation
struct BitbangI2C {
    pub gpio: GpioMem,
    delay_us: u32,
}

impl BitbangI2C {
    fn new() -> std::io::Result<Self> {
        let gpio = GpioMem::new()?;

        // Configure pins with pull-ups
        gpio.set_pullup(GPIO_SDA);
        gpio.set_pullup(GPIO_SCL);

        // Set both high (release)
        gpio.set_pin_mode(GPIO_SDA, GPIO_FSEL_INPUT);
        gpio.set_pin_mode(GPIO_SCL, GPIO_FSEL_INPUT);

        Ok(Self {
            gpio,
            delay_us: 5, // ~100kHz I2C
        })
    }

    fn delay(&self) {
        self.gpio.delay_us(self.delay_us);
    }

    fn sda_high(&self) {
        self.gpio.set_pin_mode(GPIO_SDA, GPIO_FSEL_INPUT);
    }

    fn sda_low(&self) {
        self.gpio.set_pin_mode(GPIO_SDA, GPIO_FSEL_OUTPUT);
        self.gpio.set_pin_low(GPIO_SDA);
    }

    fn scl_high(&self) {
        self.gpio.set_pin_mode(GPIO_SCL, GPIO_FSEL_INPUT);
    }

    fn scl_low(&self) {
        self.gpio.set_pin_mode(GPIO_SCL, GPIO_FSEL_OUTPUT);
        self.gpio.set_pin_low(GPIO_SCL);
    }

    fn read_sda(&self) -> bool {
        self.gpio.read_pin(GPIO_SDA)
    }

    fn start(&self) {
        self.sda_high();
        self.scl_high();
        self.delay();
        self.sda_low();
        self.delay();
        self.scl_low();
        self.delay();
    }

    fn stop(&self) {
        self.sda_low();
        self.delay();
        self.scl_high();
        self.delay();
        self.sda_high();
        self.delay();
    }

    fn write_bit(&self, bit: bool) {
        if bit {
            self.sda_high();
        } else {
            self.sda_low();
        }
        self.delay();
        self.scl_high();
        self.delay();
        self.scl_low();
    }

    fn read_bit(&self) -> bool {
        self.sda_high();
        self.delay();
        self.scl_high();
        self.delay();
        let bit = self.read_sda();
        self.scl_low();
        bit
    }

    fn write_byte(&self, byte: u8) -> bool {
        for i in (0..8).rev() {
            self.write_bit((byte >> i) & 1 != 0);
        }
        // Read ACK (low = ACK, high = NACK)
        let ack = !self.read_bit();
        ack
    }

    fn read_byte(&self, ack: bool) -> u8 {
        let mut byte = 0u8;
        for _ in 0..8 {
            byte = (byte << 1) | if self.read_bit() { 1 } else { 0 };
        }
        // Send ACK or NACK
        self.write_bit(!ack);
        byte
    }

    fn write(&self, addr: u8, data: &[u8]) -> bool {
        self.start();
        if !self.write_byte(addr << 1) {
            self.stop();
            return false;
        }
        for &b in data {
            if !self.write_byte(b) {
                self.stop();
                return false;
            }
        }
        self.stop();
        true
    }

    fn read(&self, addr: u8, buf: &mut [u8]) -> bool {
        self.start();
        if !self.write_byte((addr << 1) | 1) {
            self.stop();
            return false;
        }
        let len = buf.len();
        for (i, b) in buf.iter_mut().enumerate() {
            *b = self.read_byte(i < len - 1);
        }
        self.stop();
        true
    }

    fn write_read(&self, addr: u8, write_data: &[u8], read_buf: &mut [u8]) -> bool {
        // Write phase
        self.start();
        if !self.write_byte(addr << 1) {
            self.stop();
            return false;
        }
        for &b in write_data {
            if !self.write_byte(b) {
                self.stop();
                return false;
            }
        }

        // Repeated start for read
        self.start();
        if !self.write_byte((addr << 1) | 1) {
            self.stop();
            return false;
        }
        let len = read_buf.len();
        for (i, b) in read_buf.iter_mut().enumerate() {
            *b = self.read_byte(i < len - 1);
        }
        self.stop();
        true
    }
}

struct UInputDevice {
    file: File,
}

impl UInputDevice {
    fn new() -> std::io::Result<Self> {
        let file = OpenOptions::new()
            .read(true)
            .write(true)
            .open(UINPUT_PATH)?;

        let fd = file.as_raw_fd();

        unsafe {
            // Set event types
            ui_set_evbit(fd, EV_SYN as u64)?;
            ui_set_evbit(fd, EV_KEY as u64)?;
            ui_set_evbit(fd, EV_ABS as u64)?;

            // Set key bits
            ui_set_keybit(fd, BTN_TOUCH as u64)?;

            // Set abs bits
            ui_set_absbit(fd, ABS_X as u64)?;
            ui_set_absbit(fd, ABS_Y as u64)?;
            ui_set_absbit(fd, ABS_MT_SLOT as u64)?;
            ui_set_absbit(fd, ABS_MT_TRACKING_ID as u64)?;
            ui_set_absbit(fd, ABS_MT_POSITION_X as u64)?;
            ui_set_absbit(fd, ABS_MT_POSITION_Y as u64)?;

            // Configure ABS_X
            let abs_x = UinputAbsSetup {
                code: ABS_X,
                absinfo: AbsInfo {
                    value: 0,
                    minimum: 0,
                    maximum: SCREEN_WIDTH - 1,
                    fuzz: 0,
                    flat: 0,
                    resolution: 0,
                },
            };
            ui_abs_setup(fd, &abs_x)?;

            // Configure ABS_Y
            let abs_y = UinputAbsSetup {
                code: ABS_Y,
                absinfo: AbsInfo {
                    value: 0,
                    minimum: 0,
                    maximum: SCREEN_HEIGHT - 1,
                    fuzz: 0,
                    flat: 0,
                    resolution: 0,
                },
            };
            ui_abs_setup(fd, &abs_y)?;

            // Configure ABS_MT_SLOT
            let abs_mt_slot = UinputAbsSetup {
                code: ABS_MT_SLOT,
                absinfo: AbsInfo {
                    value: 0,
                    minimum: 0,
                    maximum: (TOUCH_POINTS - 1) as i32,
                    fuzz: 0,
                    flat: 0,
                    resolution: 0,
                },
            };
            ui_abs_setup(fd, &abs_mt_slot)?;

            // Configure ABS_MT_TRACKING_ID
            let abs_mt_tracking = UinputAbsSetup {
                code: ABS_MT_TRACKING_ID,
                absinfo: AbsInfo {
                    value: 0,
                    minimum: 0,
                    maximum: 65535,
                    fuzz: 0,
                    flat: 0,
                    resolution: 0,
                },
            };
            ui_abs_setup(fd, &abs_mt_tracking)?;

            // Configure ABS_MT_POSITION_X
            let abs_mt_x = UinputAbsSetup {
                code: ABS_MT_POSITION_X,
                absinfo: AbsInfo {
                    value: 0,
                    minimum: 0,
                    maximum: SCREEN_WIDTH - 1,
                    fuzz: 0,
                    flat: 0,
                    resolution: 0,
                },
            };
            ui_abs_setup(fd, &abs_mt_x)?;

            // Configure ABS_MT_POSITION_Y
            let abs_mt_y = UinputAbsSetup {
                code: ABS_MT_POSITION_Y,
                absinfo: AbsInfo {
                    value: 0,
                    minimum: 0,
                    maximum: SCREEN_HEIGHT - 1,
                    fuzz: 0,
                    flat: 0,
                    resolution: 0,
                },
            };
            ui_abs_setup(fd, &abs_mt_y)?;

            // Setup device
            let mut setup = UinputSetup {
                id: InputId {
                    bustype: 0x18, // BUS_I2C
                    vendor: 0x0001,
                    product: 0x0001,
                    version: 0x0001,
                },
                name: [0u8; 80],
                ff_effects_max: 0,
            };
            let name = b"HyperPixel2r Touch";
            setup.name[..name.len()].copy_from_slice(name);
            ui_dev_setup(fd, &setup)?;

            // Create device
            ui_dev_create(fd)?;
        }

        println!("Created uinput device: HyperPixel2r Touch");
        Ok(Self { file })
    }

    fn emit(&mut self, type_: u16, code: u16, value: i32) -> std::io::Result<()> {
        let event = InputEvent {
            tv_sec: 0,
            tv_usec: 0,
            type_,
            code,
            value,
        };
        let bytes = unsafe {
            std::slice::from_raw_parts(
                &event as *const _ as *const u8,
                std::mem::size_of::<InputEvent>(),
            )
        };
        self.file.write_all(bytes)
    }

    fn syn(&mut self) -> std::io::Result<()> {
        self.emit(EV_SYN, SYN_REPORT, 0)
    }

    fn report_touch(&mut self, slot: i32, id: i32, x: i32, y: i32) -> std::io::Result<()> {
        self.emit(EV_ABS, ABS_MT_SLOT, slot)?;
        self.emit(EV_ABS, ABS_MT_TRACKING_ID, id)?;
        if id >= 0 {
            self.emit(EV_ABS, ABS_MT_POSITION_X, x)?;
            self.emit(EV_ABS, ABS_MT_POSITION_Y, y)?;
        }
        Ok(())
    }

    fn report_single(&mut self, touching: bool, x: i32, y: i32) -> std::io::Result<()> {
        self.emit(EV_KEY, BTN_TOUCH, if touching { 1 } else { 0 })?;
        if touching {
            self.emit(EV_ABS, ABS_X, x)?;
            self.emit(EV_ABS, ABS_Y, y)?;
        }
        Ok(())
    }
}

impl Drop for UInputDevice {
    fn drop(&mut self) {
        unsafe {
            let _ = ui_dev_destroy(self.file.as_raw_fd());
        }
    }
}

fn read_touch_data(i2c: &BitbangI2C) -> Option<Vec<TouchPoint>> {
    // Read number of touches
    let mut buf = [0u8; 1];
    if !i2c.write_read(I2C_ADDR, &[REG_TD_STATUS], &mut buf) {
        return None;
    }
    let touch_count = (buf[0] & 0x0f) as usize;

    let mut points = Vec::new();

    if touch_count == 0 {
        return Some(points);
    }

    // Read touch data (6 bytes per touch point)
    let read_count = touch_count.min(TOUCH_POINTS);
    let mut data = vec![0u8; read_count * 6];
    if !i2c.write_read(I2C_ADDR, &[REG_TOUCH_START], &mut data) {
        return None;
    }

    for i in 0..read_count {
        let offset = i * 6;
        let event = (data[offset] >> 6) & 0x03;
        let x = (((data[offset] & 0x0f) as i32) << 8) | (data[offset + 1] as i32);
        let y = (((data[offset + 2] & 0x0f) as i32) << 8) | (data[offset + 3] as i32);
        let id = ((data[offset + 2] >> 4) & 0x0f) as i32;

        // event: 0=down, 1=up, 2=contact, 3=reserved
        let active = event != 1 && event != 3;

        points.push(TouchPoint { id, x, y, active });
    }

    Some(points)
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    println!("HyperPixel2r Touch Daemon (GPIO bitbang)");
    println!("SDA: GPIO{}, SCL: GPIO{}, INT: GPIO{}", GPIO_SDA, GPIO_SCL, GPIO_INT);
    println!("I2C Address: 0x{:02x}", I2C_ADDR);

    // Open GPIO for bitbanging
    let i2c = BitbangI2C::new()?;
    println!("GPIO initialized for I2C bitbanging");

    // Configure interrupt pin as input
    i2c.gpio.set_pin_mode(GPIO_INT, GPIO_FSEL_INPUT);

    // Test I2C communication
    let mut test_buf = [0u8; 1];
    if i2c.write_read(I2C_ADDR, &[0x00], &mut test_buf) {
        println!("Touch controller found, ID register: 0x{:02x}", test_buf[0]);
    } else {
        eprintln!("Warning: Could not read from touch controller");
    }

    // Create uinput device
    let mut uinput = UInputDevice::new()?;

    // Give udev time to set up the device
    thread::sleep(Duration::from_millis(500));
    println!("Touch daemon running (32ms polling, ~30Hz)...");

    let mut prev_points: Vec<TouchPoint> = Vec::new();
    let mut slots_active = [false; TOUCH_POINTS];
    let mut error_count = 0;

    loop {
        match read_touch_data(&i2c) {
            Some(points) => {
                error_count = 0;

                // Report multi-touch events
                for (slot, was_active) in slots_active.iter_mut().enumerate() {
                    let point = points.iter().find(|p| p.id == slot as i32);

                    match (point, *was_active) {
                        (Some(p), _) if p.active => {
                            uinput.report_touch(slot as i32, p.id, p.x, p.y)?;
                            *was_active = true;
                        }
                        (_, true) => {
                            // Touch released
                            uinput.report_touch(slot as i32, -1, 0, 0)?;
                            *was_active = false;
                        }
                        _ => {}
                    }
                }

                // Report single-touch for compatibility
                if let Some(p) = points.iter().find(|p| p.active) {
                    uinput.report_single(true, p.x, p.y)?;
                } else if prev_points.iter().any(|p| p.active) {
                    uinput.report_single(false, 0, 0)?;
                }

                uinput.syn()?;
                prev_points = points;
            }
            None => {
                error_count += 1;
                if error_count == 1 || error_count % 100 == 0 {
                    eprintln!("I2C read error (count: {})", error_count);
                }
            }
        }

        thread::sleep(Duration::from_millis(32)); // ~30Hz
    }
}
