---
title: picoCTF - Reverse Engineering | Flag Hunters
description: A writeup of picoCTF's Reeverse Engineering Flag Hunters Challenge
date: 2025-08-31
tags: ["ctf", "writeup", "reverse-engineering"]
---

[Flag Hunters](https://play.picoctf.org/practice/challenge/472?category=3&difficulty=1&page=1) is a reverse engineering challenge. The idea with these is straightforward: figure out what the program is doing, then twist that to your advantage. In this case, though, we're given the source code – so it feels less like breaking into a black box and more like an coding dry-run exercise. The code's written in Python, and our mission is to print out the flag that the program does not show us.

## Source code

```python lyric-reader.py
import re
import time


# Read in flag from file
flag = open('flag.txt', 'r').read()

secret_intro = \
'''Pico warriors rising, puzzles laid bare,
Solving each challenge with precision and flair.
With unity and skill, flags we deliver,
The ether’s ours to conquer, '''\
+ flag + '\n'


song_flag_hunters = secret_intro +\
'''

[REFRAIN]
We’re flag hunters in the ether, lighting up the grid,
No puzzle too dark, no challenge too hid.
With every exploit we trigger, every byte we decrypt,
We’re chasing that victory, and we’ll never quit.
CROWD (Singalong here!);
RETURN

[VERSE1]
Command line wizards, we’re starting it right,
Spawning shells in the terminal, hacking all night.
Scripts and searches, grep through the void,
Every keystroke, we're a cypher's envoy.
Brute force the lock or craft that regex,
Flag on the horizon, what challenge is next?

REFRAIN;

Echoes in memory, packets in trace,
Digging through the remnants to uncover with haste.
Hex and headers, carving out clues,
Resurrect the hidden, it's forensics we choose.
Disk dumps and packet dumps, follow the trail,
Buried deep in the noise, but we will prevail.

REFRAIN;

Binary sorcerers, let’s tear it apart,
Disassemble the code to reveal the dark heart.
From opcode to logic, tracing each line,
Emulate and break it, this key will be mine.
Debugging the maze, and I see through the deceit,
Patch it up right, and watch the lock release.

REFRAIN;

Ciphertext tumbling, breaking the spin,
Feistel or AES, we’re destined to win.
Frequency, padding, primes on the run,
Vigenère, RSA, cracking them for fun.
Shift the letters, matrices fall,
Decrypt that flag and hear the ether call.

REFRAIN;

SQL injection, XSS flow,
Map the backend out, let the database show.
Inspecting each cookie, fiddler in the fight,
Capturing requests, push the payload just right.
HTML's secrets, backdoors unlocked,
In the world wide labyrinth, we’re never lost.

REFRAIN;

Stack's overflowing, breaking the chain,
ROP gadget wizardry, ride it to fame.
Heap spray in silence, memory's plight,
Race the condition, crash it just right.
Shellcode ready, smashing the frame,
Control the instruction, flags call my name.

REFRAIN;

END;
'''

MAX_LINES = 100

def reader(song, startLabel):
  lip = 0
  start = 0
  refrain = 0
  refrain_return = 0
  finished = False

  # Get list of lyric lines
  song_lines = song.splitlines()

  # Find startLabel, refrain and refrain return
  for i in range(0, len(song_lines)):
    if song_lines[i] == startLabel:
      start = i + 1
    elif song_lines[i] == '[REFRAIN]':
      refrain = i + 1
    elif song_lines[i] == 'RETURN':
      refrain_return = i

  # Print lyrics
  line_count = 0
  lip = start
  while not finished and line_count < MAX_LINES:
    line_count += 1
    for line in song_lines[lip].split(';'):
      if line == '' and song_lines[lip] != '':
        continue
      if line == 'REFRAIN':
        song_lines[refrain_return] = 'RETURN ' + str(lip + 1)
        lip = refrain
      elif re.match(r"CROWD.*", line):
        crowd = input('Crowd: ')
        song_lines[lip] = 'Crowd: ' + crowd
        lip += 1
      elif re.match(r"RETURN [0-9]+", line):
        lip = int(line.split()[1])
      elif line == 'END':
        finished = True
      else:
        print(line, flush=True)
        time.sleep(0.5)
        lip += 1



reader(song_flag_hunters, '[VERSE1]')
```

Looking through the code, it's clear that the `secret_intro` is hiding the flag right at the start of the song. The catch? The lyric reader doesn’t begin there – it starts off at `[VERSE1]`. That means we never actually see the intro unless we mess with the program flow. The program moves between sections using the keywords `[REFRAIN]` and `RETURN`, so the trick is to sneak one of those into the lyrics and force the reader to jump back up to the very first line. There are 3 key areas of the code that allow us to inject our own code and make it read out the start of the song.

First up, the splitting:

```python
for line in song_lines[lip].split(';'):
```

This means every line is chopped up at `;`, so you can sneak in multiple 'mini-lines' by separating them with semicolons.

Then, the user input with no checks:

```python
elif re.match(r"CROWD.*", line):
    crowd = input('Crowd: ')
    song_lines[lip] = 'Crowd: ' + crowd
    lip += 1
```

See the problem? Whatever you type into `Crowd:` gets dumped straight back into the song. No filtering, no sanitisation – just raw text being executed by the program. Perfert to inject into.

And finally, the `RETURN` command:

```python
elif re.match(r"RETURN [0-9]+", line):
    lip = int(line.split()[1])
```

If the program sees `RETURN <number>`, it jumps directly to that line number in the song. Combine this with the input above, and we can plant something like `RETURN 0` into the lyrics and force the program to loop back to the start – right where the flag is sitting in `secret_intro`.

## Solution

The solution is to use the `Crowd:` input as our injection point. If we start our input with a `;`, the program treats everything after it as a brand-new line. That means we can slip in our own `RETURN` statement.

So, something like this works perfectly:

```
;RETURN 0
```

What happens is neat: after the jump, the program sees an empty `Crowd:` field (so nothing breaks), and then sees `RETURN 0` due to the split and immediately loops back to the top of the song. And since the `secret_intro` is sitting right there, it starts reading out the flag for us!

```
Crowd: ;RETURN 0

Echoes in memory, packets in trace,
Digging through the remnants to uncover with haste.
Hex and headers, carving out clues,
Resurrect the hidden, it's forensics we choose.
Disk dumps and packet dumps, follow the trail,
Buried deep in the noise, but we will prevail.

We’re flag hunters in the ether, lighting up the grid,
No puzzle too dark, no challenge too hid.
With every exploit we trigger, every byte we decrypt,
We’re chasing that victory, and we’ll never quit.
Crowd:
Pico warriors rising, puzzles laid bare,
Solving each challenge with precision and flair.
With unity and skill, flags we deliver,
The ether’s ours to conquer, picoCTF{70637h3r_f0r3v3r_c373964d}
```
