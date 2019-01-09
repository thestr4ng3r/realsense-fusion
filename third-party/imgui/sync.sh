
if [ "$#" -ne 1 ]; then
	echo "usage: $0 [imgui path]"
	exit 1
fi

imgui_path=$1

sync_files="LICENSE.txt imconfig.h imgui.cpp imgui.h imgui_demo.cpp imgui_draw.cpp imgui_internal.h imgui_widgets.cpp imstb_rectpack.h imstb_textedit.h imstb_truetype.h"

for f in $sync_files; do
	cp -v "$imgui_path/$f" "$f"
done

sync_files_impl="imgui_impl_glfw.h imgui_impl_glfw.cpp imgui_impl_opengl3.h imgui_impl_opengl3.cpp"

for f in $sync_files_impl; do
	cp -v "$imgui_path/examples/$f" "$f"
done
