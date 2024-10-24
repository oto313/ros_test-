
from dynaconf import Dynaconf
import os
settings = Dynaconf(
    envvar_prefix="DYNACONF",
    settings_files=['appsettings.json'],
    root_path=os.path.dirname(os.path.realpath(__file__))
)

assert settings.name == 1
# `envvar_prefix` = export envvars with `export DYNACONF_FOO=bar`.
# `settings_files` = Load these files in the order.
