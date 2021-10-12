rem @ECHO OFF       // CIから見やすいように一旦コメントアウト

rem 文字コードをutf-8に
chcp 65001

rem カレントディレクトリをバッチの場所にする
cd /d %~dp0

rem s2e root dirの１つ上へ
cd ..\..\..\
echo ### Copy S2E to temp dir
if exist ".\s2e_core_oss_temp" (
	rmdir /s /q ".\s2e_core_oss_temp"
)
xcopy /i /e /q ".\s2e_core_oss" ".\s2e_core_oss_temp"
echo ### Copy S2E build env
xcopy /i /e /q "C:\GitLab-Runner\temlate\S2E" ".\s2e_core_oss\S2E\"
echo ### Move S2E to S2E env
move ".\s2e_core_oss_temp" ".\s2e_core_oss\S2E"
cd ".\s2e_core_oss\S2E\"
rename "s2e_core_oss_temp" "s2e_core_oss"

echo ### Build with VS2019
cd ".\s2e_core_oss\"
call "C:\Program Files (x86)\Microsoft Visual Studio\2019\Community\VC\Auxiliary\Build\vcvars32.bat"
cmake -G "Visual Studio 16 2019" -A Win32  -DCMAKE_CONFIGURATION_TYPES:STRING="Debug"
cmake --build .
