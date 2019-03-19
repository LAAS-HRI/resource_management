# resource_management

Matser : [![Build Status](https://gitlab.com/laas-hri/resource_management/badges/master/pipeline.svg)](https://gitlab.com/laas-hri/resource_management/commits/master) Dev : [![Build Status](https://gitlab.com/laas-hri/resource_management/badges/dev/pipeline.svg)](https://gitlab.com/laas-hri/resource_management/commits/dev)

A basic library for implementing robot part resource management.


## Messages and buffers priorities

|          |              | 4            | 3            | 2           | 1           | 0           |
|----------|--------------|--------------|--------------|-------------|-------------|-------------|
|          |**Msg\Buffer**|**FULL FOCUS**|**PRIORITIZE**| **NORMAL**  |**SECONDARY**| **IGNORE**  |
| **4**    |**VITAL**     | 28           | 27           | 26          | 25          | 24          |
| **3**    |**URGENT**    | 23           | 18           | 17          | 16          | 15          |
| **2**    |**IMPORTANT** | 22           | 14           | 13          | 12          | 11          |
| **1**    |**HELPFUL**   | 21           | 10           | 9           | 8           | 7           |
| **0**    |**WEAK**      | 20           | 6            | 5           | 4           | 3           |
| **-1**   |**USELESS**   | 19           | 2            | 1           | 0           | -1          |
| **-2**   |**AVOID**     | -2           | -3           | -4          | -5          | -6          |
