#!/bin/bash

sudo ./groundhog/build/groundhog -pilot_only &

sleep 2

sudo ./guaca/guaca &

sudo ./cow/cow &

kst2 &
