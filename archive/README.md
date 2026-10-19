# archive/ — code kept for reference, not built

Nothing in this directory is part of the Keil project or the host tests.
It is kept because it documents how the product got here (and because the
bootloader / USB paths may come back), but it is **not** maintained.

| Path | What it was | Why it is here |
|---|---|---|
| `legacy/usb-host/` | ST USB Host CDC demo (`usbh_user.c`, `usbh_data.c`, `usbh_fs.c`, `usbhs_bsp.c` + headers) | The product uses a UART-attached 4G module, not USB. None of these files were in the Keil project; they only confused readers (and `main.c` even claimed they were compiled — they were not). |
| `legacy/ota-mqtt/` | `ota.c/h` — the old MQTT-chunked OTA | Superseded on 2026-10-02 by `app/ota/ota_http.c` (`ota.start` → HTTP + MD5). It was only reachable under `MAIN_OTA_ENABLE`, which has always been 0 since then. |
| `legacy/json-proto/` | `json_proto.c/h` | Early cloud-protocol prototype. Replaced by `cloud_proto` + `cloud_json`; no callers. |
| `legacy/llcc68-example/` | Vendor-ish LLCC68 send/receive examples | Never in the Keil project; useful as a driver reference only. |

> `legacy/bootloader/` was **moved out of the archive on 2026-10-19**: the A/B
> (mode 2) OTA test needs it again, so it now lives in `bootloader/` and is built
> by `tools/build_bl.ps1`. See `docs/OTA_DUAL_SLOT_TEST.md`.

The history is still in git (each file has its own rename commit), so
`git log --follow <path>` works if you ever need the full past.
