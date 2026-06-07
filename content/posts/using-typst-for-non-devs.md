---
title: Using Typst locally
description: How to use Typst locally without typst.app
date: 2026-06-07
tags: [tutorial, draft]
---

Typst is a typesetting system. Think LaTeX if you're a STEM nerd, or Markdown
if you're a Discord moderator. Unlike those other markup languages, Typst was
	designed to be easier to write, faster to compile, and more
	beginner-friendly. Technically, Markdown also fits these criteria, except for
	the fact that it does not compile, so we are really comparing Typst with
	LaTeX here.

Personally, it's also a replacement for word processors, with the great feature
of being plain text. There's no need for fancy, bloated, and often proprietary
applications just to read some basic text. Unfortunately, Typst does have its
own version of this: [typst.app](https://typst.app/). Which as you guessed by
the link, requires the **INTERNET!** This is really bad when I want to write my blog[^i-use-md]
under four metres of solid lead, or access my journal up in the clouds[^not-those-clouds].

I want a local environment where I can work on my Typst documents, use Git for
version control, and collaborate with others. For me, this problem is easily
solved through the use of my favourite text editor, and the only good option,
Neovim[^wheres-emacs]. However, Neovim has a learning curve, and most people
will not go out of their way to learn it. Very sad indeed, since Vim has the
best motions for writing, whether that is for prose or code.

But alas, normies exist in this world, looking at you, writers at GameDev, and
they would rather stick to their mouse for navigation than learn elite motions.
Thus, I had to explore solutions for them. I landed on Mueez Khan’s video and
decided to adapt it into written form.

<div style="text-align: center;">
	<iframe
		src="https://www.youtube.com/embed/S04WFNtNy7k"
		title="Getting started with Typst locally - Typing with Typst"
		width="560"
		height="315"
		frameborder="0"
		allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share"
		referrerpolicy="strict-origin-when-cross-origin"
		allowfullscreen>
	</iframe>
</div>

*Source video: [Getting started with Typst locally - Typing with Typst](https://www.youtube.com/watch?v=S04WFNtNy7k).*

## The bits you actually need

You only need two things:

1. [VS Codium](https://vscodium.com/), which is the telemetry-free sibling of
	VS Code.
2. The [Tiny Mist Typst](https://myriad-dreamin.github.io/tinymist/frontend/vscode.html) extension by Myriad Dream, which gives you syntax
	highlighting and a live preview.

If you already have a preferred editor and know what you are doing, you can
probably stop reading now. Honesty, I don't even know why you are reading this[^kutay-is-that-you].

## Step 1: Install VS Codium

Head over to [vscodium.com](https://vscodium.com/).

<div style="text-align: center;">
	<img src="https://vscodium.com/img/vscodium.png" alt="Screenshot of the VSCodium homepage">
</div>

From there, you have two sensible options:

1. Click the giant **Download latest release** button and grab the installer
	for your operating system and CPU architecture.
2. Use a package manager if that is more your style[^why-do-you-know-this].

Examples:

- macOS with Homebrew: `brew install --cask vscodium`
- Windows with Winget: `winget install vscodium`
- Windows with Chocolatey: `choco install vscodium`
- Windows with Scoop: `scoop install vscodium`

If you are on Windows, make sure you download the right installer. If you are
on a newer Apple machine, that usually means the ARM build. If you choose the
wrong one, your computer will not explode, but it will be annoying.

## Step 2: Install the Tiny Mist Typst extension

Open VS Codium, then click the **Extensions** tab in the left-hand sidebar.
Search for `Tiny Mist Typst` and install the one published by **Myriad Dream**.

If search fails for whatever reason, the extension page is here:
[Tiny Mist Typst on Open VSX](https://open-vsx.org/extension/myriad-dreamin/tinymist).

This extension is the bit that makes the whole thing pleasant. Without it, you
are basically just staring at raw text and pretending to be productive[^tsoding-no-lsp]. It uses
a standard called LSP (Language Server Protocol) in order to communicate with your
editor (VSCodium).

## Step 3: Make a folder for your Typst files

Click the **Explorer** tab at the top of the left sidebar and choose **Open
Folder**. Pick an existing folder or create one just for your Typst files. A
name like `typst-examples` is perfectly fine. You do not need to overthink this.

Once the folder is open, click **New File** and a file ending in `.typ`. For
example: `main.typ`.

## Step 4: Open the preview and start typing

With your new `.typ` file open, click the **Typst Preview** button (it's the magnifying-glass-style button on the right side or near
the upper-right corner with a vertical split).

That opens a live preview panel beside your editor. As you type, your document
renders immediately, which is the main reason Typst feels so much less painful
than LaTeX for normal human beings.

Try this:

```typst
I showed you my source code, pls respond
```

You should see the rendered output appear on the right almost instantly.

## Step 5: Save your work like a responsible adult

You still need to save the file yourself:

- Windows/Linux: `Ctrl + S`
- macOS: `Cmd + S`

If there is a little white dot next to the file name, it means the file has
unsaved changes. When the dot disappears, your work is actually on disk instead
of living dangerously in RAM.

## That is basically it

At this point you have a local, telemetry-free Typst setup that is much nicer
than being trapped in a browser tab. You can write documents offline, keep them
in Git, sync them however you like, and generally behave like someone with good
taste.

If all you wanted was a simple local Typst workflow without touching Neovim,
this is probably the most painless route.

[^i-use-md]: I don't use Typst for this blog at all, instead I use Markdown that gets compiled using [11ty](https://www.11ty.dev/)!
[^not-those-clouds]: I meant actual clouds, as in flying. Not the mystery "cloud" these big-tech companies offer.
[^wheres-emacs]: Yes, I see you [GNU Emacs](https://www.gnu.org/software/emacs/) users. Don't worry, I also use Emacs, so stay tuned for that post.
[^kutay-is-that-you]: Hmm, maybe your name is Kutay? If so, thank you for reading. If not, well what's your name then?
[^why-do-you-know-this]: If you know this, why are you here? Regardless thank you for reading my blog.
[^tsoding-no-lsp]: Not true, ask [rexim](https://www.youtube.com/@Tsoding).
