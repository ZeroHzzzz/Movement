#!/bin/bash

TARGET_DIR="$1"

if [ -z "$TARGET_DIR" ]; then
  echo "Please provide a target directory"
  exit 1
fi

if [ ! -d "$TARGET_DIR" ]; then
  echo "Target do not exist: $TARGET_DIR"
  exit 1
fi

find "$TARGET_DIR" -type f | while read file; do
  if file "$file" | grep -q "text"; then
    echo "Converting file: $file"
    
    iconv -f GB2312 -t UTF-8 "$file" -o "$file.tmp"
    
    if [ $? -eq 0 ]; then
      mv "$file.tmp" "$file"
    else
      echo "Converting file failed, Deteled: $file.tmp"
      rm "$file.tmp"
    fi
  fi
done

echo "Completed！"
