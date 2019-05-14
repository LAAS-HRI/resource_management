# resource_management

[![Release][Release-Image]][Release-Url]

Matser : [![Build Status](https://gitlab.com/laas-hri/resource_management/badges/master/pipeline.svg)](https://gitlab.com/laas-hri/resource_management/commits/master) Dev : [![Build Status](https://gitlab.com/laas-hri/resource_management/badges/dev/pipeline.svg)](https://gitlab.com/laas-hri/resource_management/commits/dev)

A basic library for implementing robot part resource management.


## Create your own manager

```bash
roscd resource_management; cd ../..
mkdir led_manager; cd led_manager
python3 ../resource_management/resource_management/manager_generator/generate_msgs.py --package-name led_manager_msgs --target-types Color,float32,float OnOff,bool,bool --reactive-topics emotion tagada switch
python3 ../resource_management/resource_management/manager_generator/generate.py --package-name led_manager --target-types Color,float32,float OnOff,bool,bool --reactive-topics emotion tagada switch
cd ../..
catkin_make
```

## Messages and buffers priorities

|          |              | 4            | 3            | 2           | 1           | 0            | -1          |
|----------|--------------|--------------|--------------|-------------|-------------|--------------|-------------|
|          |**Msg\Buffer**|**ATOMIC**    |**PRIORITIZE**| **NORMAL**  |**SECONDARY**|**BACKGROUND**| **INHIBIT** |
| **4**    |**VITAL**     | 24           | 23           | 22          | 21          | 20           | :x:         |
| **3**    |**URGENT**    | 19           | 15           | 14          | 13          | 12           | :x:         |
| **2**    |**HIGH**      | 18           | 11           | 10          | 9           | 8            | :x:         |
| **1**    |**STANDARD**  | 17           | 7            | 6           | 5           | 4            | :x:         |
| **0**    |**LOW**       | 16           | 3            | 2           | 1           | 0            | :x:         |
| **-1**   |**VOID**      | :x:          | :x:          | :x:         | :x:         | :x:          | :x:         |

## Create a synchronizer

```bash
roscd resource_synchronizer; cd ../..
python3 resource_management/resource_synchronizer/synchronizer_generator/generate_synchronizer.py led_resource_synchronizer led_manager:led_R led_manager:led_G led_manager:led_B
cd ..
catkin_make
```

[Release-Url]: https://github.com/LAAS-HRI/resource_management/releases
[Release-image]: http://img.shields.io/badge/release-v0.2.0-1eb0fc.svg
