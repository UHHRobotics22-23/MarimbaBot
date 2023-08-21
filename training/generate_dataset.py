import os
import shutil
def generate_dataset(dataset_path, goal_path):
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

	not_wake_words_files = [os.path.join(not_wake_words_folder, file) for file in not_wake_words_files]
	wake_words_files = [os.path.join(wake_words_folder, file) for file in wake_words_files]

	shutil.rmtree(goal_path)
	os.makedirs(os.path.join(goal_path, "not_wake_words"))
	os.makedirs(os.path.join(goal_path, "wake_words"))
	os.makedirs(os.path.join(goal_path, "test/not_wake_words"))
	os.makedirs(os.path.join(goal_path, "test/wake_words"))


	for idx, file in enumerate(not_wake_words_files):
		if file[-6:-4] == "04":
			os.system(f"cp {file} {os.path.join(goal_path, 'test', 'not_wake_words', 'not_wake_word_'+str(idx+1)+'.wav')}")
		else:
			os.system(f"cp {file} {os.path.join(goal_path, 'not_wake_words', 'not_wake_word_'+str(idx+1)+'.wav')}")

	for idx, file in enumerate(wake_words_files):
		if file[-6:-4] in ["10","09"]:
			os.system(f"cp {file} {os.path.join(goal_path, 'test', 'wake_words', 'wake_word_'+str(idx+1)+'.wav')}")
		else:
			os.system(f"cp {file} {os.path.join(goal_path, 'wake_words','wake_word_'+str(idx+1)+'.wav')}")

if __name__ == '__main__':
	generate_dataset(
		dataset_path='/home/wang/workspace/marimbabot_ws/keyword_spotting_training/dataset',
		goal_path='/home/wang/workspace/marimbabot_ws/keyword_spotting_training/training/dataset'
	)