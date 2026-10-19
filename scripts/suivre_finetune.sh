#!/usr/bin/env bash
# Suit le fine-tune sans se noyer dans la barre de progression tqdm.
L=/tmp/claude-1000/-home-genji-Osama-ws-src-mycobot-R6A/d741d405-18c8-4eee-9b31-f3cff4398cd9/scratchpad/finetune_montage0901.log
tr '\r' '\n' < "$L" | grep -Ei 'epoch [0-9]+/[0-9]+.*(loss|it/s$)|val|best|saved|Traceback|Error' | tail -${1:-20}
