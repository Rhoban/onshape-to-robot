#!/bin/bash

make html
echo "Starting watching..."
killall php
cd build/html
php -S localhost:8080 &
cd ../..

while [ true ]
do
    inotifywait source/*.rst 
    make html
    sleep 0.5
done