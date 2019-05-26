if [ ! -d ~/uavcan_vendor_spefic_types ]; then
   	echo Creating folder ~/uavcan_vendor_specific_types
	mkdir ~/uavcan_vendor_specific_types
fi
DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"
ABS_DIR=$(realpath $DIR/../lib/phoenix_msgs)
UAVCAN_DIR=$(realpath ~/uavcan_vendor_specific_types)
if [ ! -d ~/uavcan_vendor_spefic_types/phoenix_msgs ]; then
        echo Symlinking Folder $ABS_DIR to $UAVCAN_DIR
        ln -s $ABS_DIR $UAVCAN_DIR
fi
