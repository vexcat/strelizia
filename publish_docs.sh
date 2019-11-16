#!/bin/bash
doxygen strelizia-docs.cfg
mv html strel
rsync -avz strel root@ungato.tk:/var/www/ungato.tk/public_html/
mv strel html
