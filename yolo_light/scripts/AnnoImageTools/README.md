# AnnoImage Tools

This tool contains three main tools:
* image_generate.py (Python3)

 `Input:` **[background_images] , [object_images]**  
 `Output:` **[Images] , [Annotations]**  
 `Execution:` **python3 image_generate.py**

 Generate annotated dataset **(images + annotation)** with input of **background images** and background removed object images. The object will be randomly rotated, scaled, placed in the specified area or whole area on randomly picked and cropped background images. More details please see the source code.

* image_cropper.py (Python3)

 `Input:` **[Annotations] , [Images]**  
 `Output:`  **[Images]**  
 `Execution:` **python3 image_cropper.py /path/to/annotation/folder /path/to/image/folder /path/to/output/folder float(PERCENTAGE_OF_CROP)**

 Cropped image through annotation file. It generate the images by cropped out the annotation in annotation files. The generated files name will separate with the classes. PERCENTAGE_OF_CROP is controlling how big you need to crop. **1.0** will be the same size of annotation.

* image_marker.py (Python3)

 `Input:` **[Annotations] , [Images]**  
 `Output:`  **[Images]**  
 `Execution:` **python3 image_marker.py /path/to/annotation/folder /path/to/image/folder /path/to/output/folder float(PERCENTAGE_OF_MARKING)**

 Mark out the bounding box through annotation file. It copy the images with bounding box marked. PERCENTAGE_OF_MARKING is controlling how big you need to mark. **1.0** will be the same size of annotation.
