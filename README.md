Experimental Linux fork for running it on MSD7816/MSD7818 (kernel 6.7.0-rc6)

[.config](https://gist.github.com/maniekx86/abbbdef38c84eae7efed8118df665ca3) that I used to compile and run this

Use mipsel toolchain to compile this. Also I recommend to check arch/mips/boot/dts/mstar/msd7816-test.dts too

Default bootargs: `console=ttyS0,115200 root=/dev/sda2 rootwait clk_ignore_unused rw fbcon=font:TER16x32`
```
Linux kernel
============

There are several guides for kernel developers and users. These guides can
be rendered in a number of formats, like HTML and PDF. Please read
Documentation/admin-guide/README.rst first.

In order to build the documentation, use ``make htmldocs`` or
``make pdfdocs``.  The formatted documentation can also be read online at:

    https://www.kernel.org/doc/html/latest/

There are various text files in the Documentation/ subdirectory,
several of them using the Restructured Text markup notation.

Please read the Documentation/process/changes.rst file, as it contains the
requirements for building and running the kernel, and information about
the problems which may result by upgrading your kernel.
```
