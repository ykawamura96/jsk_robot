#!/bin/bash

ln -s  $PWD/python/bCAPClient  $HOME/.local/lib/python3.6/site-packages/bcap_client

pip3 install -r $PWD/python/requirements.txt
