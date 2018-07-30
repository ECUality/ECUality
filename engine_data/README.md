# Engine Map Format

The following is an example of how engine map data is reported to the serial monitor. This format, unfortunately, is a bit hard to read and debug.
```
n_air
n_rpm
Air gridlines: a1 a2 a3
rpm gridlines: r1 r2 r3 r4
m1,1	m1,2	m1,3
m2,1	m2,2	m2,3
m3,1	m3,2	m3,3
m4,1	m4,2	m4,3
```

To turn the data-reporting format into a table for human-consumption in, for example, Excel, use the following transformation.

||a1|a2|a3|
|---|---|---|---|
|r1|m1,1|m1,2|m1,3|
|r2|m2,1|m2,2|m2,3|
|r3|m3,1|m3,2|m3,3|
|r4|m4,1|m4,2|m4,3|
