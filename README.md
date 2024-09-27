## Weed_Plucking_using_Deep_Learning_With-_Respberrypi--4.


## What is this project about ?


Weeds are the unwanted crops in agriculture lands. They disturb the growing plants around them. Weeds are dependent plants which use nutrients, water, etc resources. So we need to remove them as early as possible. So this project will help to detect the weed in between plants further we can add a mechanical arm which will automatically pick weed and remove them. 


## What will this give you ?

We can use this code to detect weeds in agriculture lands.


## How i did it ?

I used **YOLO(You Only Look Once)** Real time object detection algorithm to detect weeds. Dataset i used is taken from [kaggle](https://www.kaggle.com/) and dataset name is *crop and weed detection data with bounding boxes* ([Link to dataset](https://www.kaggle.com/ravirajsinh45/crop-and-weed-detection-data-with-bounding-boxes)) which contains around *1300 images*. Traning is done in [Goole Colab](https://colab.research.google.com/).

## How to acess the repositort ?
Structure of the repository

```
├───testing
│   ├───images
│   └───results
└───traning
    ├───backup
    └───test
```

- Traning folder consists of files used for traning
- Testing folder consists of testing file

Clone the repo and Upload Traning file in google drive and open ipynb file in colab and then you can change the parameters and you can play with the code.

For testing and for using the project download weights from the [link](https://mega.nz/file/LIcFWZhb#XZ9YACBuAz2jeiklyqiDN1AGyDbfvztacRIlar9wP7k) and paste it in testing folder.

For testing project we have to run the code.py file in the thony python ide in respberry pi make sure all the connections has done as per the circuit daigram with webcame access.


