# resource_management

Matser : [![Build Status](https://gitlab.com/laas-hri/resource_management/badges/master/pipeline.svg)](https://gitlab.com/laas-hri/resource_management/commits/master) Dev : [![Build Status](https://gitlab.com/laas-hri/resource_management/badges/dev/pipeline.svg)](https://gitlab.com/laas-hri/resource_management/commits/dev)

A basic library for implementing robot part resource management.


## Messages and buffers priorities

|          |              | 4            | 3            | 2           | 1           | 0           |
|----------|--------------|--------------|--------------|-------------|-------------|-------------|
|          |**Msg\Buffer**|**FULL FOCUS**|**PRIORITIZE**| **NORMAL**  |**SECONDARY**| **IGNORE**  |
| **4**    |**VITAL**     | 25           | 24           | 23          | 22          | 21          |
| **3**    |**URGENT**    | 20           | 16           | 15          | 14          | 13          |
| **2**    |**IMPORTANT** | 19           | 12           | 11          | 10          | 9           |
| **1**    |**HELPFUL**   | 18           | 8            | 7           | 6           | 5           |
| **0**    |**WEAK**      | 17           | 4            | 3           | 2           | 1           |
| **-1**   |**USELESS**   | 0            | -1           | -2          | -3          | -4          |
| **-2**   |**AVOID**     | -5           | -6           | -7          | -8          | -9          |
