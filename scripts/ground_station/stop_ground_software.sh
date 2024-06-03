#!/bin/bash

# Stop suite of ground station command and control software

kill -10 $(pgrep kst2);
kill -10 $(pgrep guaca);
kill -10 $(pgrep groundhog);
kill -10 $(pgrep cow);
kill -10 $(pgrep owl);
