# Multi Object Checker

Multi Object Checker (MOC) is a tool for collaborative, web-based image annotation.

## Main Usage

* Place the image data in `./makingDataset/dataset_folder/batch_folder`. Multiple dataset folders and batch folders are permitted. Names of these subfolders can be set freely.
* The results are saved in `./results`. Please do not forget to press the "s" key to save.
* Other operations are self-explanatory. Click the "Help" button to access more information.

## Additional Scripts

The tool is provided with two additional Python 3 scripts in `./scritps` for optional pre/post-processing of the data and results.

`./scripts/dataset-raw2moc.py`

This Python 3 script needs two specific subfolders to run:
* `./scripts/dataset-raw`: Input images in JPG, PNG or BMP format.
* `./scripts/dataset-moc`: Output resized/cropped images in PNG format.

`./scripts/annotation-moc2voc.py`

This Python 3 script needs three specific subfolders to run:
* `./scripts/dataset-moc`: Input images in PNG format.
* `./scripts/annotation-moc`: Input annotations in MOC/TXT format.
* `./scripts/annotation-voc`: Output annotations in VOC/XML format.

## Credits

Original MOC by Panasonic, scripts and improvements by Team NAIST-Panasonic.
