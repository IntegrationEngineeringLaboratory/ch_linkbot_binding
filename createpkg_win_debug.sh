#!/bin/sh
PACKAGE=chbarobo
VERSION=2.0.1
ARCH=Win32
PKGDIR=$PACKAGE-$VERSION-$ARCH/$PACKAGE

echo Building $PACKAGE-$VERSION-$ARCH.zip ...
rm -rf $PACKAGE-$VERSION-$ARCH
rm -rf $PACKAGE-$VERSION-$ARCH.zip
mkdir -p $PKGDIR
mkdir $PKGDIR/demos
mkdir $PKGDIR/docs
mkdir $PKGDIR/bin
mkdir $PKGDIR/dl
cp -R chbarobo/lib $PKGDIR/lib
#cp -R chbarobo/dl $PKGDIR/dl
cp build/src/Debug/liblinkbot.dl $PKGDIR/dl
cp -R chbarobo/include $PKGDIR/include 
#cp dlls/$ARCH/* $PKGDIR/bin
zip -rq $PACKAGE-$VERSION-$ARCH.zip $PACKAGE-$VERSION-$ARCH
