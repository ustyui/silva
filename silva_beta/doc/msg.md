# The meanings of seq and msgid

| seq | meaning | msgid | meaning | overall meaning |
----|----|----|----|----
|  0  | default |   0   | default |    default      |
|  1  |  idle   |   1   | blink   | idle blink motion |
|  3  |  slave  |   1   | debug   | slave debug gui message|
|  3  |  slave  |   2   | HSM     | humanoid simulation model csv input|
|  4  |  auto   |   0   |  abs    | absolute value to auto memory|
|  4  |  auto   |   1   | relative| relative value, need to combine with bases|
|  4  |  auto   |   2   |  bias   | bias value with the current value|

