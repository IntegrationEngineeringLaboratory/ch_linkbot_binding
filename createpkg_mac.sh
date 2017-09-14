#!/bin/sh
PACKAGE=chbarobo
VERSION=2.0.0
ARCH=Mac-Intel
PKGDIR=$PACKAGE-$VERSION-$ARCH/$PACKAGE

echo Building $PACKAGE-$VERSION-$ARCH.zip ...
rm -rf $PACKAGE-$VERSION-$ARCH
rm -rf $PACKAGE-$VERSION-$ARCH.zip
mkdir -p $PKGDIR
mkdir $PKGDIR/lib
mkdir $PKGDIR/demos
mkdir $PKGDIR/dl
mkdir $PKGDIR/include
mkdir $PKGDIR/docs
mkdir $PKGDIR/bin
cp chlinkboti.chf $PKGDIR/lib
cp chlinkbotl.chf $PKGDIR/lib
cp liblinkbot.dl $PKGDIR/dl
cp linkbot.h $PKGDIR/include
#cp dlls/$ARCH/* $PKGDIR/bin
zip -rq $PACKAGE-$VERSION-$ARCH.zip $PACKAGE-$VERSION-$ARCH
