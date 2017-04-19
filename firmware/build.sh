mkdir temp_build

rm `basename $1`.bin
rm temp_build/*.*
cp $1/*.* temp_build

cp libraries2/*.h temp_build
cp libraries2/*.cpp temp_build
cp libraries2/*.c temp_build

particle compile photon temp_build --saveTo `basename $1`.bin
