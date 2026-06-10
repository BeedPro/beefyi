---
title: I use Emacs BTW
description: Using vanilla GNU Emacs with vanilla keybinds
date: 2026-06-10
tags: [ "tools", "writing"]
---

### I thought you used neovim?

Yes, I do use Neovim, and I still intend to use it for all my coding and writing
that does not involve non-English or non-Latin scripts. I am not abandoning the
editor just because some UTF-8 font decided to have a nervous breakdown inside
my terminal. For plain English, code, Markdown, Typst, and anything else that is
supported, Neovim remains home. I do have some opinions about how computing is
still painfully Eurocentric, and how we probably need better standards and
tooling for languages that are not English wearing a fake moustache. But that is
a political post for another day (don't worry, I am going to write a political
post at some point in the future, so stay tuned![^not-into-politics]).

So the question becomes: what cursed niche problem was powerful enough to make
me change editors?

Arabic.

I am learning Arabic, and because apparently learning vocabulary, grammar, and a
new writing system was not enough, I also want to learn how to touch-type
it. Since it is now the 21st century and everything has been reduced to typing
into glowing rectangles, I figured I should probably learn how to type the
language properly too.

To be clear, I still barely know any vocabulary or grammar. This is not me
claiming fluency. This is me adding another small, consistent habit to the pile,
because language learning is mostly just doing tiny annoying things every day
until your brain gives up and accepts them.

Of course, I can still practise handwriting with good old paper and a nice
pencil, like a civilised person. But we no longer live in the golden age of
letters, notebooks, and pretending your handwriting is not slowly becoming worse.
Most of what I will actually write will probably be typed.

Unfortunately, Arabic needs proper font support, right-to-left behaviour, and a
writing environment that does not make me want to debug my terminal for three
hours. Which means I needed something with a GUI.

Tragic, I know.

### Then, why not [INSERT-NAME] editor?

The answer is simple: I need FLOSS[^whats-FLOSS] software, good keybindings, and
proper customisability.

The FLOSS part matters because I do not want my writing environment to be
someone else's product strategy. I want software I can inspect, modify, share,
and keep using without wondering when a company will decide that basic text
editing now requires an account, telemetry, AI credits, or a monthly
subscription called "Writer Pro Ultra Plus".

The keybindings matter because I do not want to learn a completely isolated
system that only works inside one application. I want bindings that are
well-established, widely supported, and useful in other editors or environments.
The whole point is to build muscle memory that travels well, not trap myself in
a bespoke keyboard dungeon.

And then there is customisability, which is where Emacs starts looking less like
a weird old editor and more like a suspiciously powerful text-shaped operating
system. I can change the behaviour, the fonts, the layout, the modes, the
bindings, and all the tiny annoying details that usually make me abandon a tool
after twenty minutes.

So no, this is not about finding the prettiest editor, the newest editor, or the
one with the cleanest landing page and the most tastefully animated screenshots.
I need something free, open, keyboard-friendly, and deeply customisable.

Unfortunately for everyone involved, that points rather directly at Emacs.

### Anything else?

Emacs is a very large rabbit hole. Not a cute little rabbit hole either. This is
one of those suspicious holes in the ground with stairs, wiring, a mailing list,
and someone at the bottom explaining why their configuration is actually a
personal computing environment.

But I am not trying to learn all of Emacs at once, because I still value the
small amount of peace I have left. Emacs can be an editor, a file manager, a Git
client, an email client, a terminal, a calendar, an IRC client, and probably a
toaster if someone has written enough Elisp. That is far too much power to hand
to a person who only wanted Arabic font rendering.

So the particular hole I have chosen is writing and notes. That is the part of
Emacs that actually interests me right now: prose, journalling, study notes,
language practice, and anything else that benefits from being plain text but
slightly more organised than a folder full of files called `notes-final-2.txt`.

And once you start talking about writing and notes in Emacs, there is one thing
you cannot avoid.

Org mode.

Org mode is the part of Emacs that everyone brings up with the same energy as a
person trying to get you into a cult, except annoyingly, they may have a point.
It is used for notes, outlines, TODO lists, documents, agendas, literate
programming, exporting, planning, journalling, and probably remembering where you
left your keys.

For my use case, the appeal is simple: I want plain-text notes that can grow
with me. Something simple enough for quick Arabic practice, but powerful enough
that I can later organise lessons, vocabulary, grammar notes, writing exercises,
blog ideas, and whatever other nonsense I convince myself is productivity.

I am not going to fully explain Org mode here, because that deserves its own
post, and because any attempt to explain Org mode briefly is how you accidentally
write a book. I will probably make a separate post about how I use it for notes,
writing, language learning, and pretending my life is more organised than it
actually is.

### Vanilla keybinds???

Yes, I use the vanilla keybindings. No Evil mode, no Vim emulation, no attempt
to turn Emacs into Neovim wearing a trench coat. Just normal, traditional,
pinkie-threatening Emacs bindings, as the elders intended.

There is a reason for this beyond self-inflicted suffering. Emacs-style
keybindings show up in more places than people realise. You can find them in
terminals, shells, readline-based prompts, minibuffers, text fields, and random
programs that quietly decided `C-a` and `C-e` were good ideas. Learning the
default bindings means the muscle memory is not trapped inside one editor. It
travels.

Also, it is not quite as painful as it sounds. I use home-row mods and a split
keyboard[^cutting-keyboards], so pressing `Control` and `Meta` is not the same
hand-contorting ritual it would be on a normal keyboard made by someone who
apparently hates fingers. My layout does a lot of the ergonomic heavy lifting,
which makes vanilla Emacs bindings feel much more reasonable.

So yes, I am using the default bindings on purpose. Partly for portability,
partly for consistency, and partly because once your keyboard is weird enough,
Emacs starts seeming normal.


[^not-into-politics]: Not into politics? congratulations, that is politics.
[^whats-FLOSS]: Free, Libre and Open Source Software.
[^cutting-keyboards]: No, I did not cut my keyboard in half. I will show you
    about my hardware setup later ;)
