---
title: picoCTF - Cryptography | EVEN RSA CAN BE BROKEN???
description: A writeup of picoCTF's Cryptography EVEN RSA CAN BE BROKEN??? Challenge
date: 2025-08-31
tags: ["ctf", "writeup", "cryptography"]
---

Picking up from the last blog, I had a go at the [EVEN RSA CAN BE BROKEN??](https://play.picoctf.org/practice/challenge/470?category=2&difficulty=1&page=1) cryptography challenge. To be honest, it felt more like an exercise in RSA than a proper puzzle, but it's a good way to dip your toes into crypto. Think of RSA as the "hello world" of cryptography - simple, classic, and a solid starting point. The challenge kicks off by asking us to connect via netcat, where we're handed an encrypted message `c`, and its public key `(N, e)`.

```bash
> nc verbal-sleep.picoctf.net 56957

N: 22715301090244944555782064618276252525023771968883364484388038636123683358976283740304327683722247637952999616864052130853477842537404917091331688687558122
e: 65537
cyphertext: 14293253600340155854054831280856936574632994961947887972916917485609072959383679102282010013872401143755895229136463678205587523830693631363813380495455347
```

I won't go into how RSA works here - that's left to some [further reading](https://en.wikipedia.org/wiki/RSA_cryptosystem). But the short version (TL;DR) is this: RSA relies on a pair of keys - one public and one private. Together, they're what let us encrypt and decrypt ciphertext (term for encrypted text). As you can see we only got the public key, and so this challenge is to find the private key and then decrypt the message.

The catch that allows us to find the private key is to notice that $N$ is _even_. Typically $N$ is the product of two **large primes** - but it is always a product of primes. If it's even, that means one of those primes must be $2$.

So we set $p = 2$, and then grab the other prime by dividing: $q = N /p$. From there, we calculate Euler’s totient, $\phi = (p - 1)(q - 1)$.

Next step: finding the private key exponent $d$. This is just the modular inverse of $e$ with respect to $\phi$. We can calculate this using Python's `pow(e, -1, phi)`.

With $d$ in hand, we can finally decrypt: `m = pow(c, d, N)`. That gives us the message, but in integer form. To make sense of it, we work out how many bytes long it should be, convert it with `to_bytes`, and then decode to ASCII.

And that's really all there is to it. Merging all the steps together, the full script looks like this:

```python
N = 22715301090244944555782064618276252525023771968883364484388038636123683358976283740304327683722247637952999616864052130853477842537404917091331688687558122
e = 65537
c = 14293253600340155854054831280856936574632994961947887972916917485609072959383679102282010013872401143755895229136463678205587523830693631363813380495455347
p = 2
q = N // p
phi = (p - 1) * (q - 1)
d = pow(e, -1, phi)
m = pow(c, d, N)
byte_len = (m.bit_length() + 7) // 8
pt_bytes = m.to_bytes(byte_len, "big")

print("d =", d)
print("m (int) =", m)
print("m (hex) =", pt_bytes.hex())
print("m (ascii) =", pt_bytes.decode("utf-8"))
```

Giving us the decrypted message:

```
picoCTF{tw0_1$_pr!m3df98b648}
```
