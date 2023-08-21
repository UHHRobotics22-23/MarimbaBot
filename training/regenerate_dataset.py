import os
import shutil
def generate_dataset(dataset_path, goal_path,file_name_prefix):
	goal_path = os.path.join(goal_path, file_name_prefix)
	not_wake_words_folder = os.path.join(dataset_path, "not_wake_words")
	wake_words_folder = os.path.join(dataset_path, "wake_words")

	not_wake_words_files_path = []
	wake_words_files_path = []

	not_wake_words_files = os.listdir(not_wake_words_folder)
	wake_words_files_number = len(os.listdir(wake_words_folder))
	for i in range(wake_words_files_number):
		not_wake_words_files = os.listdir(os.path.join(not_wake_words_folder, str(i+1)))
		wake_words_files = os.listdir(os.path.join(wake_words_folder, str(i+1)))
		not_wake_words_files_path += [os.path.join(not_wake_words_folder, str(i+1), file) for file in not_wake_words_files]
		wake_words_files_path += [os.path.join(wake_words_folder, str(i+1), file) for file in wake_words_files]



	try:
		shutil.rmtree(goal_path)
	except:
		pass
	os.makedirs(os.path.join(goal_path, "not-wake-word"))
	os.makedirs(os.path.join(goal_path, "wake-word"))
	os.makedirs(os.path.join(goal_path, "test/not-wake-word"))
	os.makedirs(os.path.join(goal_path, "test/wake-word"))

	idx = 0
	for _ , file in enumerate(not_wake_words_files_path):
		if file[-6:-4] == "04":
			os.system(f"cp {file} {os.path.join(goal_path, 'test', 'not-wake-word', file_name_prefix+'.'+str(idx+1)+'.wav')}")
			idx+=1
		else:
			os.system(f"cp {file} {os.path.join(goal_path, 'not-wake-word', file_name_prefix+'.'+str(idx+1)+'.wav')}")
			idx += 1
	for _, file in enumerate(wake_words_files_path):
		if file[-6:-4] in ["10","09"]:
			os.system(f"cp {file} {os.path.join(goal_path, 'test', 'wake-word', file_name_prefix+'.'+str(idx+1)+'.wav')}")
			idx += 1
		else:
			os.system(f"cp {file} {os.path.join(goal_path, 'wake-word', file_name_prefix+'.'+str(idx+1)+'.wav')}")
			idx += 1
if __name__ == '__main__':
	generate_dataset(
		file_name_prefix='hi-marimbabot',
		dataset_path='/home/wang/workspace/marimbabot_ws/keyword_spotting_training/dataset',
		goal_path='/home/wang/workspace/marimbabot_ws/keyword_spotting_training/training/'
	)