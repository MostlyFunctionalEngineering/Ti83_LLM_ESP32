# TIGPT — Claude AI on a TI-83 Plus

Shoves an ESP32 inside a TI-83+ calculator and lets it talk to Claude via the link port. Type a question on the calc, get an answer back on the screen. It's JANKY AF to get this to work on a Ti-83 Plus, I HIGHLY RECOMMEND DOING OTHER PEOPLE'S CODE ON A TI-84. The Ti-84's communication protocol is much more robust, supported, and tested than my crappy code. USE AT YOUR OWN CAUTION!

This involves soldering to the back of your calculator's PCBA, so there is a high liklihood of screwing something up. PROCEED AT YOUR OWN RISK!

Lastly, a thing about ethics. I didn't make this so that you can cheat on your tests. It's nowhere near reliable enough to do that, nor should you count on it to be. To be honest, whatever you're learning is probably less work than getting this to work, and will do you more good. 

So don't cheat, it's not worth it. 

---

## How it works

The TI-83+ has a 2.5mm link port (TIP + RING) that TI used for calculator-to-calculator transfers and the old TI-Graph Link cable. It's an open-drain, bit-banged, ~9600 baud effective protocol. 

The XIAO ESP32-C3 sits inside the back of the calculator wired to the link port pins. When you run the TIGPT program on the calc, it sends your query over the link port to the ESP32, which forwards it to the Anthropic API over WiFi and sends the response back. 

I wasn't able to get the Ti "silent protocol" to work (I'm a mechanical engineer not a software guy), but I did manage to get the "remote control" code working. All communications happen via the ESP-32 executing remote commands (like hitting the buttons but over software). It makes it look like a ghost is controlling the calculator, it's fun, janky, but works!

You send your question, and it sends back an AI answer!

---

## Hardware

- Seeed XIAO ESP32-C3
- TI-83 Plus (the silver one, not the SE — haven't tested others)
- 4 wires
- Some way to power the ESP32 (I tapped the calc's battery rails)

**Wiring:**

| ESP32-C3 pin | Link port |
|---|---|
| GPIO 6 | TIP (sleeve closest to tip) |
| GPIO 5 | RING (middle sleeve) |
| GND | GND (base sleeve) |
| VUSB | (5V power from the batterises) |

The link port is 3.3V logic so it plays nice with the ESP32 directly. I think there are some 3.3V pins on the back of the PCB too, so those would be probably better than grabbing the batter voltage directly. 

---

## Software setup

### PlatformIO dependencies

Add to your `platformio.ini`: (I added these, but just make sure they're there).

```ini
lib_deps =
    bblanchon/ArduinoJson @ ^7.0.0
    tzapu/WiFiManager @ ^2.0.17
```

### Claude API Key

I used claude's Haiku API to get this to work. I won't put instructions here on how to get an API key, but it does invovle like a $5 minimum token purchase. We'll be having pretty small token usage, so $5 would buy you a crap ton of queries. Anyway, snag that API key, stash it somewhere safe, and then have it ready for when you do the first boot up. 

### First boot

1. Flash the firmware (if you need to check which port it's in, run **`ls /dev/ttyA*`**)
2. The ESP32 will broadcast a WiFi network called **`Ti83-Plus`**
3. Connect your phone to it — a captive portal should pop up automatically (or go to `192.168.4.1`) NOTE: I highly recommend setting the wifi network to whatever your phone's hotspot is so that you can use the calculator in a number of different places, not just your home network. I did add a way to get the captive portal to open up after initial config, but it's probably better to just set it up correctly the first time. 
4. Enter your preferred WiFi network (e.g. phone hotspot like I mentioned above) + password, and your Anthropic API key (`sk-ant-...`)
5. Save — the ESP32 reboots, connects to your WiFi, and is ready
6. Be sure to hit `EXIT` on the web portal or it will just stay open and be buggy. 

Credentials are stored in NVS (flash) and survive power cycles.


### Installing the BASIC program

From the calculator home screen, send the value `69` (nice) as variable `A` to the ESP32. If you don't know how to save variables to letters, ask your teacher or pay more attention in class. Then, send the magic code `A` by going to  `2ND → LINK → 9:REAL → SELECT A → TRANSMIT → ENTER`. The ESP32 will detect the magic number and automatically push the TIGPT program to the calculator, then open the PRGM menu for you.

---

## Using TIGPT

Run `prgmTIGPT` from the PRGM menu.

- **ASK** — type your query, press ENTER. The calc sends it to the ESP32 and waits.
- **SHOW** — displays the last response from Claude across up to 8 rows (128 chars max)
- **EXIT** — exits

The whole round trip takes about 5-10 seconds depending on WiFi and API response time. Don't touch the screen while it does it's thing (Remember it works by remoting in to the calculator and pushing buttons just like you do, and it doesn't know if you pushed a different button in the meantime). The ESP32 code should handle hanging situations with timeouts, but I didn't test it much tbh. 

---

## A NOTE ABOUT DEEP SLEEP

To save power, since this bad chicken is always connected to the batteries, the easiest way would be to to just remove a AAA battery when you're not using it. That's annoying, so I added a "deep sleep". This was mad vibe-coded, so it's also extra janky. Deep sleep prevents the ESP32 from responding to basically anything. I have it set so that if you don't talk to it for 10 minutes, it goes into deep sleep mode. Then, it wakes up briefly every 1 minute, checks for a sent message, then goes back to sleep. To wake it up, just send anything over to the calculator via the program `TIGPT`. I usually just send `TEST` or anything really. It's janky, because you just have to keep sending it until you catch it when it's awake, but I can usually catch it within the 3rd or 4th try at most. If you missed it, it'll just say `DONE` and not do anything. If you do catch it, it will remote in and run `TIGPT` and send your query like it should. Then it'll be awake as long as you talk to it again within the next 10 mintues. 

A proper implementation would use light sleep with a GPIO wakeup on the TIP/RING pins so the ESP32 can respond instantly to any link activity. That's a more involved rewrite though and I ran out of steam — this is good enough for the battery life improvement without completely breaking the user experience. PRs welcome I guess.

Note to self: come back and make this better at some point, this sucks. 


## Reconfiguring WiFi or API key

From the calc home screen, type `"WIFI"` and save it as a string variable and send it to the ESP32 via the link menu. The ESP32 will finish the transfer, then launch the config portal again as `Ti83-Plus`. Connect your phone and update whatever you need. Be sure to hit `EXIT` on the web portal or it will just stay open and be buggy. 

---

## TI-83+ Link Protocol notes

The protocol is documented at [merthsoft.com/linkguide/ti83+](https://merthsoft.com/linkguide/ti83+/indexex2.html). A few things that took forever to figure out and aren't obvious from the docs:

- The ESP32 uses machine ID `0x73` for manual transfers (same as the calculator itself). Using `0x23` (documented "computer" ID) doesn't work reliably here.
- `Send()` from BASIC sends an RTS but never completes the transfer properly — we use it purely as a trigger signal and ignore the rest of the handshake.
- TI string bytes are *tokens*, not ASCII. Space is `0x29`, period is `0x3A`, division is `0x83`. If you just pass ASCII through, you get `QuartReg` and `CubicReg` showing up in your responses.
- `sub()` in BASIC throws `ERR:INVALID DIM` if you ask it to pull a 16-char chunk starting past the end of the string — so the display logic guards each row with `If length(Str1)>=N*16` before calling `sub()`.

---

## Power consumption

| Mode | Current draw |
|---|---|
| Active (WiFi connected, no sleep) | ~80mA |
| Modem sleep (what this uses at idle) | ~17-20mA |
| Deep sleep | ~5µA |

With modem sleep enabled (via `esp_wifi_set_ps(WIFI_PS_MAX_MODEM)`), idle current drops to around 17mA. On the 4xAAA setup that's a couple days of always-on.

---

## File structure

```
src/main.cpp        — all firmware (it's one file, deal with it)
platformio.ini      — build config
README.md           — this file
```

---

## Known issues / TODO

- Deep sleep wakeup is timer-only. If the calc sends something while sleeping, it times out on its end. User just has to retry.
- The `remoteSendStr0()` delays are tuned for my specific calculator. If link navigation is flaky, bump the `delay(600)` values after `kLinkIO` and `kCapB` up to 800-1000ms.
- `claudeResponse` is only 64 bytes but Claude is asked for 128 chars. This is a latent bug that doesn't bite often because the sanitizer tends to compress things, but it should be 129 bytes.
- WiFiManager's `setSaveParamsCallback` doesn't always fire on `autoConnect` — the API key reload from NVS after connect is a workaround for this.
- No OTA updates. Reflashing requires USB access, which means opening the calculator. Plan accordingly.
- The Ti Protocol is weird and different than ASCII, so there are all sorts of hex codes for characters that pop up randomly that aren't what they're supposed to be. I usually fixed them as they came up, but they still keep showing up. Like colons `:` or apostraphes `'` will show up as some of the weird Ti symbols. You can usually piece together what the AI is trying to say though. 

---

## License

Do whatever you want with this, idc it's mostly non-functional anyways. 