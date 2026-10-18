@echo off
REM Re-run the OTA host tests at other optimization levels (close to the
REM firmware's AC6 -O1/-Os).
REM
REM Why a .cmd and not a .ps1: passing these arrays through inline PowerShell
REM mangles the args (it shows up as "linker cannot find 0"). Batch has no
REM such ambiguity. Keep the comments ASCII-only too -- cmd.exe reads this
REM file as ANSI, so non-ASCII REM text gets executed as commands.
REM
REM Usage (from the demo dir): _host_test\opt_recheck.cmd

setlocal
set WARN=-Wall -Wextra -Wno-unused-parameter -Wno-type-limits -Wmaybe-uninitialized -Wstrict-overflow=2 -Warray-bounds=2
set INC=-std=c99 -I_host_test/stub -Iapp/links/g4 -Iapp/links/lora -Iapp/links/serial -Iapp/cloud -Iapp/model -Iapp/service -Iapp/common -Iapp/common/ring_buf -Iapp/ota -Iapp/cjson -Iapp/drivers/lora -I_host_test -Iinc
set SRC=app/ota/ota_http.c app/ota/md5.c app/links/g4/g4_io.c app/links/g4/g4_urc.c app/common/ring_buf/ring_buf.c _host_test/host_stub.c _host_test/fake_ota_port.c _host_test/test_ota_http.c

for %%O in (O2 Os O3) do (
  echo === -%%O ===
  gcc -%%O %WARN% %INC% %SRC% -o _host_test\_opt_%%O.exe
  if errorlevel 1 (
    echo   %%O : BUILD FAILED
  ) else (
    _host_test\_opt_%%O.exe > nul
    if errorlevel 1 (
      echo   %%O : RUN FAILED
    ) else (
      echo   %%O : PASS
    )
    del /q _host_test\_opt_%%O.exe > nul 2>&1
  )
)
endlocal
