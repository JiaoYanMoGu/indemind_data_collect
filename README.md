## indemind_data_collect
Simple program to collect data from indemind camera



### Required
+ `OpenCV`: for write image

### Build
```bash
mkdir build
cmake ..
make
```
### Usage
```bash
# connect the camera  in usb 3.0 port
mkdir data
cd data
sudo ../indemind_collect_data -c ../config/config.yaml
```


### Problems
> The code is a simple practice, there are some problems,such as:
> + the data saved belong to root 
> + the last few frames of the data may be broken because of Ctrl+C to quit.

