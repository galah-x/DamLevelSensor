#!/usr/bin/perl

while (<>) {
  if (/Temp=([\d\.]+)C/)
    { $temp = $1;
    }
  if (/depth=([\d\.]+)/)
    { $depth = $1;
      print "$temp $depth\n"
    }
}
