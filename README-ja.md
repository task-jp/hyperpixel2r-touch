# hyperpixel2r-touch

Raspberry Pi 用 Pimoroni HyperPixel 2r 丸型ディスプレイのユーザースペースタッチ入力デーモン。

## 概要

このデーモンは FT5x06 静電容量式タッチコントローラーから GPIO ビットバンギング I2C 経由でタッチデータを読み取り、uinput を使用して仮想 Linux 入力デバイスを作成します。マルチタッチプロトコル B を使用して最大 2 点の同時タッチをサポートします。

## 特徴

- GPIO ビットバンギング I2C による直接制御 - カーネルドライバー不要
- マルチタッチプロトコル B 対応（2 タッチポイント）
- 互換性のためのシングルタッチフォールバック
- 最小限の依存関係（libc + nix）
- 小さなバイナリサイズ（strip 後 約 100KB）

## 必要条件

### ハードウェア

- Raspberry Pi（Pi Zero 2 W で動作確認済み）
- Pimoroni HyperPixel 2r 丸型ディスプレイ（480x480）

### ソフトウェア

- `vc4-kms-dpi-hyperpixel2r` オーバーレイを含む Linux カーネル
- `/dev/gpiomem` アクセス権（gpio グループまたは root）
- `/dev/uinput` アクセス権（input グループまたは root）

## インストール

### ビルド済みバイナリ

[Releases](https://github.com/task-jp/hyperpixel2r-touch/releases) からダウンロード。

### ソースからビルド

```bash
# ネイティブビルド（Raspberry Pi 上）
cargo build --release

# クロスコンパイル（x86_64 ホストから）
cargo build --release --target aarch64-unknown-linux-gnu
```

## 設定

### デバイスツリーオーバーレイ

HyperPixel 2r ディスプレイオーバーレイが必要です：

```
# /boot/firmware/config.txt
dtoverlay=vc4-kms-v3d
dtoverlay=vc4-kms-dpi-hyperpixel2r
```

### Systemd サービス

```bash
sudo cp hyperpixel2r-touch /usr/local/bin/
sudo cp hyperpixel2r-touch.service /etc/systemd/system/

sudo systemctl daemon-reload
sudo systemctl enable hyperpixel2r-touch
sudo systemctl start hyperpixel2r-touch
```

### 手動実行

```bash
sudo ./hyperpixel2r-touch
```

入力デバイスの確認：

```bash
cat /proc/bus/input/devices | grep -A5 HyperPixel
```

## 技術詳細

| パラメータ | 値 |
|-----------|-------|
| タッチコントローラー | FT5x06（EDT-FT5x06 互換） |
| I2C アドレス | 0x15 |
| GPIO SDA | GPIO10 |
| GPIO SCL | GPIO11 |
| GPIO INT | GPIO27 |
| 画面解像度 | 480x480 |
| 最大タッチポイント | 2 |
| ポーリングレート | 約 30Hz |

## 動作原理

1. **GPIO ビットバンギング**: `/dev/gpiomem` を使用して GPIO ピンを直接制御し、カーネル I2C ドライバーをバイパスして I2C 通信を行います。

2. **タッチデータ読み取り**: 約 32ms ごとに FT5x06 タッチコントローラーをポーリングしてタッチ状態と座標を読み取ります。

3. **UInput デバイス**: アプリケーションが標準的なタッチスクリーンと同様に使用できる仮想入力デバイスを作成します。

## トラブルシューティング

### "Could not open /dev/gpiomem"

ユーザーを gpio グループに追加：
```bash
sudo usermod -a -G gpio $USER
```

### "Could not open /dev/uinput"

ユーザーを input グループに追加：
```bash
sudo usermod -a -G input $USER
```

または sudo で実行。

### "I2C read error"

- HyperPixel 2r が正しく接続されているか確認
- ディスプレイオーバーレイがロードされているか確認：`dmesg | grep -i hyperpixel`

## ライセンス

MIT License - [LICENSE](LICENSE) ファイルを参照。

## 関連プロジェクト

- [Pimoroni HyperPixel 2r](https://shop.pimoroni.com/products/hyperpixel-2-1-round)
- [vc4-kms-dpi-hyperpixel2r overlay](https://github.com/raspberrypi/linux)
