# after you virtusl environment activated, then excute this script
export LD_LIBRARY_PATH=$(dirname $(python -c "import nvidia.cudnn;print(nvidia.cudnn.__file__)")):$LD_LIBRARY_PATH
