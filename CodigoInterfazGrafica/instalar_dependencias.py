import subprocess
import pkg_resources
import urllib.request
import re

def esta_instalado(pkg_str):
    try:
        req = pkg_resources.Requirement.parse(pkg_str)
        dist = pkg_resources.get_distribution(req.name)
        return dist.version == req.specs[0][1] if req.specs else True
    except Exception:
        return False

def buscar_wheel_en_piwheels(paquete, version=None):
    base_url = f"https://www.piwheels.org/simple/{paquete.lower()}/"
    try:
        with urllib.request.urlopen(base_url) as response:
            html = response.read().decode("utf-8")
            pattern = fr'href="([^"]+{paquete.lower()}[-_]{version}[^"]+\.whl)"' if version else r'href="([^"]+\.whl)"'
            matches = re.findall(pattern, html, re.IGNORECASE)
            if matches:
                return base_url + matches[0]
    except:
        pass
    return None

with open("requirements.txt", encoding="utf-8-sig") as f:
    for line in f:
        pkg = line.strip()
        if not pkg or pkg.startswith("#"):
            continue

        # Extraer nombre y versión si hay ==
        if "==" in pkg:
            nombre, version = pkg.split("==")
        else:
            nombre, version = pkg, None

        if esta_instalado(pkg):
            print(f"✅ Ya está instalado: {pkg}")
            continue

        print(f"\n🔍 Buscando rueda precompilada para {pkg} en piwheels...")
        url_whl = buscar_wheel_en_piwheels(nombre, version)

        try:
            if url_whl:
                print(f"🌐 Encontrado .whl precompilado: {url_whl}")
                subprocess.check_call(["python3", "-m", "pip", "install", url_whl])
            else:
                print(f"⚠️  No se encontró rueda, instalando con pip normal: {pkg}")
                subprocess.check_call(["python3", "-m", "pip", "install", pkg])
        except subprocess.CalledProcessError:
            print(f"❌ Falló la instalación de: {pkg}")
