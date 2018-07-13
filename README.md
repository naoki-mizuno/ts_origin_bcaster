# ts_origin_bcaster

Calculates the origin of the Total Station in the GNSS coordinate system by
measuring three points on the ground with the Total Station and GNSS.


## Usage

### Requirements

- `gdal`: In Ubuntu, provided as `python-gdal` or `python3-gdal` package

```
$ roslaunch ts_origin_bcaster ts_origin_bcaster_node.launch
```

Take a look at `config/sample.yaml` to see how to use it.


## License

MIT


## Author

Naoki Mizuno (mizuno.naoki@rm.is.tohoku.ac.jp)
