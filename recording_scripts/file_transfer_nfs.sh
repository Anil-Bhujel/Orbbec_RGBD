#!/bin/bash

# Define source and destination paths
# LOCAL_FILE="/home/dcs/workspaces/isaac_ros-dev/test_file.txt"  # Replace with the local file path
# REMOTE_DIR="/mnt/nfs_share/test"         # The NFS mounted directory
# LOG_FILE="/home/dcs/workspaces/isaac_ros-dev/file_transfer_log/log_file.log" # log file
#!/bin/bash

#!/bin/bash

# Define source and destination directories
SOURCE_DIR="/home/dcs/workspaces/isaac_ros-dev/bags"
DEST_DIR="/mnt/nfs_share/bags"  # The NFS mount point

# Log file to record the progress
LOG_FILE="/home/dcs/workspaces/isaac_ros-dev/file_transfer_log/file_transfer_log.txt"

# Create log file if it doesn't exist
if [ ! -e "$LOG_FILE" ]; then
    touch "$LOG_FILE"
fi

# Function to move files and clean up
move_and_clean() {
    local src_dir="$1"
    local dest_dir="$2"

    # Log the start of the transfer
    echo "Starting transfer for directory: $src_dir" | tee -a "$LOG_FILE"

    # Check if the source is a directory
    if [ ! -d "$src_dir" ]; then
        echo "Error: $src_dir is not a valid directory!" | tee -a "$LOG_FILE"
        return
    fi

    # Loop through all directories under the source directory
    for dirs in "$src_dir"/*/; do
        if [ -d "$dirs" ]; then
            for dir in "$dirs"/*/; do
                if [ -d "$dir" ]; then
                    # Find the most recently created directory
                    last_created_dir=$(find "$dir" -mindepth 1 -maxdepth 1 -type d | sort | tail -n 1)
                    # last_created_dir=$(basename "$last_created_dir") # To separate the relative path of dir
                    
                    # Loop through each subdirectory and move it (except the most recent one)
                    for item in "$dir"/*; do
                        # echo $(basename "$item")
                        
                        if [ -d "$item" ] && [ $(basename "$item") != $(basename "$last_created_dir") ]; then
                            # To provide relative path of source directory in destination 
                            relative_path=$(realpath --relative-to="$src_dir" "$dir")
                            target_dir="$dest_dir/$relative_path" 

                            # Log and move the directory
                            echo "Moving $item to $target_dir" | tee -a "$LOG_FILE"

                            # Ensure the target directory does not already exist on the destination
                            if [ ! -d "$target_dir" ]; then
                                sudo mkdir -p "$target_dir"
                                
                            fi

                            # Move the directory
                            sudo mv "$item" "$target_dir"

                            # Check if the transfer was successful
                            if [ $? -eq 0 ]; then
                                echo "Successfully moved $item to $target_dir" | tee -a "$LOG_FILE"

                                # After successful transfer, delete the source directory
                                echo "Cleaning up directory $item in source" | tee -a "$LOG_FILE"
                                sudo rm -rf "$item"

                                echo "Cleanup completed for $item" | tee -a "$LOG_FILE"
                            else
                                echo "Error moving $item to $target_dir" | tee -a "$LOG_FILE"
                            fi
                        fi
                    done
                    
                fi
            done
        fi
    done
}

# Main execution

echo "Starting file transfer at $(date)" | tee -a "$LOG_FILE"

# Call the move_and_clean function
move_and_clean "$SOURCE_DIR" "$DEST_DIR"

echo "File transfer and cleanup completed at $(date)" | tee -a "$LOG_FILE"

######### END #################
