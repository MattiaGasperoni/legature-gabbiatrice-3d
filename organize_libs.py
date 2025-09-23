import os
import shutil
import xml.etree.ElementTree as ET

# === CONFIG ===
VCXPROJ_FILE = 'LegatureGabbiatrice.vcxproj'
DEST_INCLUDE = 'include'
DEST_LIB = 'lib'
DEST_BIN = 'bin'
EXT_HEADERS = ['.h', '.hpp']
EXT_LIBS = ['.lib']
EXT_DLLS = ['.dll']
DRY_RUN = True  # True = solo stampa; False = copia i file

# === UTILS ===

def safe_copy(src, dst_dir):
    try:
        if not os.path.isfile(src):
            return False
        os.makedirs(dst_dir, exist_ok=True)
        dst = os.path.join(dst_dir, os.path.basename(src))
        if not os.path.isfile(dst):
            shutil.copy2(src, dst)
            print(f"✔️ Copiato: {src} → {dst}")
        else:
            print(f"➖ Già presente: {dst}")
        return True
    except Exception as e:
        print(f"❌ Errore copiando {src}: {e}")
        return False

def collect_files_in_folder(folder, exts):
    found = []
    for root, _, files in os.walk(folder):
        for file in files:
            if any(file.lower().endswith(ext) for ext in exts):
                found.append(os.path.join(root, file))
    return found

def parse_paths_from_xml(file_path, tags):
    """Estrae percorsi da un file XML per i tag specificati (es. AdditionalIncludeDirectories)."""
    paths = set()
    try:
        tree = ET.parse(file_path)
        root = tree.getroot()
        ns = {'ms': 'http://schemas.microsoft.com/developer/msbuild/2003'}

        for tag in tags:
            for elem in root.findall(f".//ms:{tag}", ns):
                raw_text = elem.text or ""
                for path in raw_text.split(';'):
                    path = path.strip()
                    if path and not path.startswith('$') and os.path.isdir(path):
                        paths.add(os.path.normpath(path))
    except Exception as e:
        print(f"⚠️  Errore leggendo {file_path}: {e}")
    return paths

# === MAIN ===

def main():
    if not os.path.exists(VCXPROJ_FILE):
        print(f"❌ File non trovato: {VCXPROJ_FILE}")
        return

    include_dirs = set()
    lib_dirs = set()

    # 1. Percorsi dal .vcxproj
    include_dirs |= parse_paths_from_xml(VCXPROJ_FILE, ['AdditionalIncludeDirectories'])
    lib_dirs     |= parse_paths_from_xml(VCXPROJ_FILE, ['AdditionalLibraryDirectories'])

    # 2. Percorsi dal .props (assumiamo "resources/MyLibs.props")
    props_path = os.path.join("resources", "MyLibs.props")
    if os.path.isfile(props_path):
        include_dirs |= parse_paths_from_xml(props_path, ['AdditionalIncludeDirectories'])
        lib_dirs     |= parse_paths_from_xml(props_path, ['AdditionalLibraryDirectories'])
    else:
        print(f"⚠️  File MyLibs.props non trovato in: {props_path}")

    print(f"\n📂 Trovate {len(include_dirs)} cartelle include")
    print(f"📦 Trovate {len(lib_dirs)} cartelle lib\n")

    # 3. Copia gli header
    print("📄 Copia header...")
    for inc_dir in sorted(include_dirs):
        headers = collect_files_in_folder(inc_dir, EXT_HEADERS)
        for h in headers:
            if not DRY_RUN:
                safe_copy(h, DEST_INCLUDE)

    # 4. Copia le .lib
    print("\n📚 Copia .lib...")
    for lib_dir in sorted(lib_dirs):
        libs = collect_files_in_folder(lib_dir, EXT_LIBS)
        for lib in libs:
            if not DRY_RUN:
                safe_copy(lib, DEST_LIB)

    # 5. Copia le .dll
    print("\n🔗 Cerco e copio .dll...")
    dll_dirs = include_dirs.union(lib_dirs)
    for dll_dir in sorted(dll_dirs):
        dlls = collect_files_in_folder(dll_dir, EXT_DLLS)
        for dll in dlls:
            if not DRY_RUN:
                safe_copy(dll, DEST_BIN)

    print("\n✅ Tutto finito!")

if __name__ == '__main__':
    main()
