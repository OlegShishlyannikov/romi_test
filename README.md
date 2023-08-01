# romi_test

In first console:

```bash
sudo socat -d -d pty,link=/dev/ttyVA00,echo=0,perm=0777 pty,link=/dev/ttyVB00,echo=0,perm=0777 &
sudo ./romi_test -d /dev/ttyVB00 -b 115200
```

Second console:

```bash
cat > /dev/ttyVA00
```
