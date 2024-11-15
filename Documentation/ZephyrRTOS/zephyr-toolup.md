Goal
* Be able to run a driver sample as a [freestanding application](https://docs.zephyrproject.org/latest/develop/application/index.html#zephyr-freestanding-application)
* Run sample for Chargers API with [`ti,bq24190`](https://docs.zephyrproject.org/latest/build/dts/api/bindings/charger/ti,bq24190.html)on i2c `reg = <0x6B>;`

Preconditions
* Review page 17 of [BQ24195 datasheet](https://www.ti.com/lit/ds/symlink/bq24195.pdf)
* Battery charging is enabled by I2C register bit (`REG01[5:4]`) = 01 and CE is low

Methodology
For context, `zephyrproject/zephyr/samples/drivers/charger/src/main.c` has the following DeviceTree device node that needs to be set:
```c
static const struct device *chgdev = DEVICE_DT_GET(DT_NODELABEL(charger));
```

To make a Zephyr freestanding application, a proper zephyr repo instance is accessible but separate from an `app/` directory as shown below:
```md
<home>/
├─── zephyrproject/
│     ├─── .west/
│     │    └─── config
│     ├── zephyr/
│     ├── bootloader/
│     ├── modules/
│     └── ...
│
└─── app/
     ├── CMakeLists.txt
     ├── prj.conf
     ├── boards
         └── esp32c3_devkitm.overlay
     └── src/
         └── main.c
```

The contents of custom device tree overlay overriding `chgdev` for the `boards/esp32c3_devkitm.overlay` with app name `app` is shown below:
```c
&i2c0 {
	status = "okay";
	charger: bq24190@6b {
		compatible = "ti,bq24190";
		reg = <0x6B>; // I2C address of the BQ24195
		constant-charge-current-max-microamp = <2000000>;
		constant-charge-voltage-max-microvolt = <4200000>; 
	};
};
```


```c
cmake_minimum_required(VERSION 3.20.0)

set(BOARD esp32c3_devkitm)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})

project(
	BMS-SMPS
	DESCRIPTION "Evaluation project for zephyr,charger API"
	LANGUAGES C
)

target_sources(
	app
	PRIVATE
	src/main.c
)
```

```conf
CONFIG_MAIN_STACK_SIZE=4096

# Enable console support
CONFIG_CONSOLE=y
CONFIG_UART_CONSOLE=y

# Enable GPIO support
CONFIG_GPIO=y

# Enable I2C for the ESP32-C3 Devkit
CONFIG_I2C=y
CONFIG_I2C_INIT_PRIORITY=60
  
# Enable the Charger API
CONFIG_CHARGER=y
# Set up the specific driver
CONFIG_CHARGER_BQ24190=y

# Enable logging (optional, for debugging)
CONFIG_LOG=y
CONFIG_TRACING_SYSCALL=y
CONFIG_LOG_DEFAULT_LEVEL=3
```

Postconditions
* Use an overlay file to set chgdev for using the `zephyr,charger` API sample.
* Put the BQ24193 into `CHARGER_STATUS_CHARGING` to charge an attached single 1.65 V 18650 cell and successfully run the `zephyr,charger` on an ESP32-C3 DEVKITM `--board esp32c3_devkitm`.
* Can run the following commands to build and flash:


```bash
# See https://github.com/zephyrproject-rtos/zephyr/pull/80342/
west init -m https://github.com/rriveramcrus/zephyr
west build
west flash
west espressif monitor
```

