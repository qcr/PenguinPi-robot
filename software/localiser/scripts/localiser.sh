#!/bin/bash
echo "Starting localiser"
localiser &
echo $! > /var/www/penguinpi/localiser.pid