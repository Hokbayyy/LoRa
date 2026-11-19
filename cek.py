    #!/usr/bin/env python3
    """
    =============================================================================
    Tes Komunikasi AT RAK3272S (UART2) via Raspberry Pi 5 (UART3: /dev/ttyAMA3)
    =============================================================================
    """

    import serial
    import time
    import sys

    PORT = '/dev/ttyAMA3'
    BAUDRATE = 115200

    print("=" * 65)
    print(f"📡 MENGUJI AT DENGAN RAK3272S PADA {PORT} ({BAUDRATE} bps)...")
    print("📌 Pastikan kabel terhubung:")
    print("   - Pin 24 Pi 5 (TXD3) -> Pin RX2 RAK3272S (Header J4 Pin 8)")
    print("   - Pin 21 Pi 5 (RXD3) -> Pin TX2 RAK3272S (Header J4 Pin 7)")
    print("   - Pin 1 Pi 5 (3.3V)  -> Pin 3V3 RAK3272S (Header J5 Pin 9)")
    print("   - Pin 6 Pi 5 (GND)   -> Pin GND RAK3272S (Header J4 Pin 4)")
    print("=" * 65)

    try:
        ser = serial.Serial(
            port=PORT,
            baudrate=BAUDRATE,
            timeout=2.0,
            rtscts=False,
            dsrdtr=False,
            xonxoff=False
        )

        ser.reset_input_buffer()
        ser.reset_output_buffer()
        time.sleep(0.2)

        print("1. Mengirim: AT\\r\\n")
        ser.write(b"AT\r\n")
        ser.flush()

        print("2. Menunggu respon balasan...")
        start = time.time()
        response = b""

        while (time.time() - start) < 2.0:
            if ser.in_waiting > 0:
                chunk = ser.read(ser.in_waiting)
                response += chunk
                if b"OK" in response:
                    break
            time.sleep(0.02)

        print("-" * 65)
        print(f"Raw Data Diterima ({len(response)} byte): {repr(response)}")
        text = response.decode('utf-8', errors='replace').strip()
        print(f"Teks Diterima: {repr(text)}")
        print("-" * 65)

        if b"OK" in response or "OK" in text:
            print("🎉 BERHASIL! RAK3272S merespon 'OK' via UART3 (/dev/ttyAMA3)!")
        else:
            print("⚠️ Belum ada respon OK. Coba tukar kabel Pin 24 dan Pin 21.")

        ser.close()

    except Exception as e:
        print(f"❌ Error membuka port {PORT}: {e}")

    print("=" * 65)
