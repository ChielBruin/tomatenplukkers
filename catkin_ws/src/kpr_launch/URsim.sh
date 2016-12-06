#!/bin/sh
URSIM_DIR=$1

echo $URSIM_DIR
cd $URSIM_DIR
./URControl&
./start-ursim.sh

