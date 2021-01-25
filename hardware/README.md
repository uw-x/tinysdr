# Overview
<img src="../images/tinysdr_routes.png" alt="TinySDR PCB All Layers" width="400" align="right"/>

TinySDR hardware designed with a six layers FR4 PCB material. We need six layer to implement multiple components with complex power plains that gives us flexibility to control power consumption in various scenarios.

We place RF paths on the top layer with impedance control for Sub-GHz and 2.4 GHz frequencies.

This PCB is originally designed with Altium Designer version 19 and most recently rendered by version 20.1.11. All the libraries that required to build the PCB are included in [PCB files](./pcb-tinysdr.zip).

## Layers
After routing wires for all components we place polygons on each layer. In table below we show all the polygons and their purpose in our design.

| Layer       | Polygons Description |Image  |
| ----------- | ------------------- |-------|
| Top | Connected to GND.<br /> Multiple Polygons.<br /> Includes impedance control for both RF paths and LVDS connections. | <img src="../images/tinysdr_top.png" alt="Top Layer Polygon" width="300" align="center"/>|
| Inner 1 | Connected to GND. | <img src="../images/tinysdr_mid1.png" alt="Inner Layer 1 Polygon" width="300" align="center"/>|
| Inner 2 | Connected to GND. | <img src="../images/tinysdr_mid2.png" alt="Inner Layer 2 Polygon" width="300" align="center"/>|
| Inner 3 | Connected to GND. | <img src="../images/tinysdr_mid3.png" alt="Inner Layer 3 Polygon" width="300" align="center"/>|
| Inner 4 | Connected to Power.<br /> Includes multiple polygons for various power planes. | <img src="../images/tinysdr_mid4.png" alt="Inner Layer 4 Polygons" width="300" align="center"/>|
| Bottom | Connected to GND. | <img src="../images/tinysdr_bottom.png" alt="Bottom Layer Polygon" width="300" align="center"/>|

## How to Use
To use PCB files in this directory follow these steps:
- Install the latest version of the [Altium Designer](https://www.altium.com/products/downloads)
- Download and extract [PCB files](./pcb-tinysdr.zip)
- Open project in Altium Designer

Note: When you run design rule check you will see few erros regarding the `silk gaps` which you can ignore.
