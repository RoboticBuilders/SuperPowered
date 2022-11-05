#!/bin/zsh

for i in {1..6}; do echo Creation FullRun-$i.py; sed -E "s/(.*)doRunWithTiming\(.*\)/\1doRunWithTiming(_myrun$i)/;s/# LEGO .*$/# LEGO type:standard slot:$i/" FullRun.py > FullRun-$i.py; echo Done; done

