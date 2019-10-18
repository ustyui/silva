# Modules
```
import modules.<funciton> as YY
XX = YY.<classes>/<def>
```

## motion
Usage:
### set_angles(dict angles)
  return (dict formatted_angles)
  
The input data of [dict angles] should be like below:
```
angles = {0: 30, 1:20, 5:10 ...}
```
The key is the joint that you may want to change the angle; The value is the deg value.

```
modules.motion.set_angles(angles)
```

### set_angles_to(dict angles, 0/1/2/3)
  ``publish angles to final output, through channel idle/reflex/slave/auto, corresponding to no.0/1/2/3.''
  
To set angles to slave node, e.g.:
```
XX.set_angles_to(angles, 2)
```
### reset()
  ``reset channel 99 input.''
```
XX.reset()
```

## utils


## topics
