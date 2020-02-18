ESP32 S/PDIF driver
======================

This is the S/PDIF driver for ESP32.

The driver files are follows.

* spdif.h
* spdif.c

The three APIs are provided.

* `void spdif_init(int rate)`
* `void spdif_write(const void *src, size_t size)`
* `void spdif_set_sample_rates(int rate)`

# Example

The driver project includes modified version of a2dp_sink example to use the S/PDIF driver.

# Hardware Required

The S/PDIF toslink transmitter is needed.
The default data output pin is GPIO27. You can change the pin to use "idf.py menuconfig".
![circuit](/spdif_circuit.png "circuit")
![board](/spdif_board.jpg "board")

# Configure the project

```
idf.py menuconfig
```

# Build and Flash

```
idf.py -p PORT flash monitor
```

(To exit the serial monitor, type ``Ctrl-]``.)
