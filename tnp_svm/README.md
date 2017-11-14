**Item recognition using HOG-SVM**

#30 minutes training at the Competition

1. receive the 20or36 items list

2. take a picture of unknown items, remove background, crop the items

3. choose 10or16 known items from pre-augmented 40 known items and copy them to the for_training_stow/pick folder

4. crop, rectify orientation and augment unknown items data 
	and train the all (known+unknown) data
	
	augment_train_stow/pick.sh 

	(python augmentaion.py 'data_dir_path' 'load_group_name' 'save_group_name')
	python augmentation.py /root/share/tnp_svm amazon_bgr augmented_stow/pick
	python augmentation.py /root/share/tnp_svm own_bgr augmented_stow/pick

	(python HOG_SVM_training.py 'data_dir_path' 'load_group_name' 'save_model_name')

	python HOG_SVM_training.py /root/share/tnp_svm augmented_stow hog_svm_stow/pick.pkl

	augmented_stow----avery_binder
				    |_burts_and
				    |_epsom	 

check the filename of the model (save_model_name)!!!


##preparation

pip install "scikit-learn==0.17.1"

pip install scikit-image

pip install scipy

pip install scikit-image

pip install pandas

########
execute:
########
python hog_svm_recognition_node.py

or

roslaunch tnp_svm test_svm_recognition.launch

call:

rosservice call /tnp_svm/recognize_by_svm "target_item_id:
  data: '0'" (I don't use target_item_id in the node now.)


To test without using service from rec-space

roslaunch tnp_svm test_svm_recognition.launch

rosservice call /tnp_svm/test_recognize_by_svm ...



(If we augmentate and train all data(known+unknown))
python Trainfor30minutes.py 'data directory path'


