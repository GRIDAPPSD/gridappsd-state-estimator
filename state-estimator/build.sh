#!/bin/sh

jsonpath="../../json/single_include/nlohmann"
sspath="../../suitesparse-metis-for-windows/SuiteSparse"
ssconfpath="$sspath/SuiteSparse_config"
cspath="$sspath/CXSparse"
amdpath="$sspath/AMD"
btfpath="$sspath/BTF"
colamdpath="$sspath/COLAMD"
klupath="$sspath/KLU"
sepath="."

amqpath="/usr/local/include/activemq-cpp-3.9.4"
aprpath="/usr/local/apr/include/apr-1"

include="-I $jsonpath -I $ssconfpath -I $cspath/Include -I $amdpath/Include -I $btfpath/Include -I $colamdpath/Include -I $klupath/Include -I $sepath/include"
echo include: $include


echo --- Compile config ---
gcc -c -std=c99 $include $ssconfpath/SuiteSparse_config.c

echo --- Complie CS ---
gcc -c -std=c99 $include $cspath/Source/*.c

echo --- Compile AMD ---
gcc -c -std=c99 $include $amdpath/Source/*.c

echo --- Compile BTF ---
gcc -c -std=c99 $include $btfpath/Source/*.c

echo --- Compile COLAMD ---
gcc -c -std=c99 $include $colamdpath/Source/*.c

echo --- Compile KLU ---
gcc -c -std=c99 $include $klupath/Source/*.c

echo --- Complie nlohmann/json ---
g++ -c -std=c++11 $jsonpath/json.hpp

echo --- Compile SE ---
g++ -Wno-deprecated-declarations -c -std=c++11 $include -I$amqpath -I$aprpath $sepath/src/state-estimator.cpp

rm cs_convert.o

echo --- Link ---
g++ *.o -l activemq-cpp -l stdc++ -l ssl -o bin/state-estimator.out

echo --- Cleanup ---
rm *.o


# demo.exe
#bin/state-estimator.out [simid]
