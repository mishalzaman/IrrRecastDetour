#!/bin/bash

# Define the target directory
TARGET_DIR="/home/aaron/Dev/IrrRecastDetour/assets/realms/ROTH_Resources"

# Check if the directory exists
if [ ! -d "$TARGET_DIR" ]; then
    echo "Error: Directory $TARGET_DIR does not exist."
    exit 1
fi

echo "Scanning $TARGET_DIR for files with spaces..."

# Find all files (-type f) containing a space (-name "* *")
# Use -depth so we handle files before the directories they sit in (good practice)
find "$TARGET_DIR" -depth -name "* *" -type f | while read -r FILE; do
    # Get the directory path and the filename separately
    DIR=$(dirname "$FILE")
    FILENAME=$(basename "$FILE")

    # Replace spaces with underscores in the filename
    NEW_FILENAME="${FILENAME// /_}"

    # Construct the full new path
    NEW_PATH="$DIR/$NEW_FILENAME"

    echo "Renaming: '$FILENAME' -> '$NEW_FILENAME'"
    
    # Perform the rename
    mv "$FILE" "$NEW_PATH"
done

echo "Done!"
