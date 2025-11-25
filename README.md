## Deploying RNNoise on Alif E7 DK/AK RTSS-HE

From the root of the workspace:

```bash
cd zephyr
west build -b alif_e7_dk_rtss_he ../alif/samples/modules/tflite-micro/rnnoise/ -p always

The compiled firmware will be generated at:
build/zephyr/zephyr.bin
