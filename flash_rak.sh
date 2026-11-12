#!/bin/bash
# =============================================================================
# Tool Pembuka Kunci UART1 RAK3272S Menggunakan arduino-cli di Ubuntu CLI
# =============================================================================

set -e

PORT="/dev/ttyAMA3"

echo "================================================================="
echo "🚀 PEMBUKA KUNCI UART1 RAK3272S (Mode AT Permanen)"
echo "   Menjalankan Arduino di Ubuntu CLI (Tanpa Butuh Desktop/GUI)"
echo "================================================================="
echo ""
echo "⚠️  PERHATIAN KONEKSI SEMENTARA (HANYA UNTUK PROSES FLASH INI):"
echo "   Modul RAK menerima upload firmware lewat port UART2 (Header J4):"
echo "   - Pi 5 Pin 24 (TX) -> Hubungkan sementara ke Header J4 Pin 8 (RX2)"
echo "   - Pi 5 Pin 21 (RX) -> Hubungkan sementara ke Header J4 Pin 7 (TX2)"
echo "   - Daya 3.3V & GND tetap terhubung."
echo "================================================================="
echo ""

# 1. Cek apakah arduino-cli sudah terpasang
if ! command -v arduino-cli &> /dev/null; then
    echo "📦 [1/5] Mengunduh dan memasang arduino-cli untuk Ubuntu ARM64..."
    curl -fsSL https://raw.githubusercontent.com/arduino/arduino-cli/master/install.sh | sh
    sudo mv bin/arduino-cli /usr/local/bin/
    rm -rf bin
    echo "   ✅ arduino-cli berhasil dipasang!"
else
    echo "   ✅ arduino-cli sudah terpasang di sistem."
fi

# 2. Konfigurasi Board Manager RAKwireless
echo "⚙️  [2/5] Mengatur Board Manager RAKwireless RUI3..."
if [ ! -f ~/.arduino15/arduino-cli.yaml ]; then
    arduino-cli config init || true
fi
arduino-cli config add board_manager.additional_urls https://raw.githubusercontent.com/RAKWireless/RAKwireless-Arduino-BSP-Index/main/package_rakwireless_com_rui_index.json || true
arduino-cli core update-index

# 3. Pasang Core STM32 RUI3 jika belum ada
if ! arduino-cli core list | grep -q "rak_rui:stm32"; then
    echo "📥 [3/5] Mengunduh & memasang Core RAK RUI3 STM32 (butuh waktu 1-2 menit)..."
    arduino-cli core install rak_rui:stm32
else
    echo "   ✅ Core RAK RUI3 STM32 sudah terpasang."
fi

# Cari FQBN Board
FQBN=$(arduino-cli board listall rak_rui | grep -i "3172" | head -n 1 | awk '{print $NF}')
if [ -z "$FQBN" ]; then
    FQBN="rak_rui:stm32:WisDuoRAK3172EvaluationBoard"
fi
echo "   🎯 Target Board FQBN: $FQBN"

# 4. Buat Sketch Pembuka Kunci UART1
SKETCH_DIR="$HOME/enable_uart1"
mkdir -p "$SKETCH_DIR"
cat << 'EOF' > "$SKETCH_DIR/enable_uart1.ino"
void setup() {
  Serial.begin(115200);
  Serial1.begin(115200, RAK_AT_MODE);
}

void loop() {
}
EOF
echo "📝 [4/5] Sketch pembuka kunci UART1 berhasil disiapkan di $SKETCH_DIR/enable_uart1.ino"

# 5. Compile Sketch
echo "🔨 [5/5] Meng-compile sketch menjadi firmware..."
arduino-cli compile --fqbn "$FQBN" "$SKETCH_DIR"
echo "   ✅ Kompilasi sukses!"

echo ""
echo "================================================================="
echo "👉 SIAP MENG-UPLOAD KE MODUL RAK3272S!"
echo "   Pastikan kabel Pi 5 terhubung sementara ke Header J4 (UART2)!"
echo "================================================================="
read -p "Tekan ENTER jika kabel sudah siap untuk mulai upload..."

echo "⚡ Meng-upload firmware ke modul pada port $PORT..."
arduino-cli upload -p "$PORT" --fqbn "$FQBN" "$SKETCH_DIR"

echo ""
echo "================================================================="
echo "🎉 BERHASIL 100%! UART1 MODUL RAK SEKARANG SUDAH PERMANEN DALAM MODE AT!"
echo "================================================================="
echo "Langkah selanjutnya:"
echo "1. Cabut kabel sementara dari Header J4."
echo "2. Pasang modul kembali ke jalur PCB Anda (Header J5 UART1)."
echo "3. Jalankan 'python3 cek.py' untuk memverifikasi!"
echo "================================================================="
