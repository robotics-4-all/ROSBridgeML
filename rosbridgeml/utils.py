from os.path import dirname, join
from textx import metamodel_from_file
import textx.scoping.providers as scoping_providers

CURRENT_FILE_DIR = dirname(__file__)
GRAMMAR_DIR = join(CURRENT_FILE_DIR, "grammar")


def get_mm(debug=False, global_scope=True):
    mm= metamodel_from_file(
        join(GRAMMAR_DIR, 'rosbridge.tx'),
        global_repository=global_scope,
        debug=debug
    )

    def importURI_to_scope_name(import_obj):
        # this method is responsible to deduce the module name in the
        # language from the importURI string
        # e.g. here: import "file.ext" --> module name "file".
        return import_obj.importURI.split('.')[0]

    def conv(i):
        return i.replace(".", "/") + ".rbr"

    mm.register_scope_providers(
        {
            "*.*": scoping_providers.FQNImportURI(
                importAs=True,
                # importURI_to_scope_name=importURI_to_scope_name
                # importURI_converter=conv
            )
        }
    )

    return mm


def build_model(model_fpath):
    mm = get_mm(global_scope=True)
    model = mm.model_from_file(model_fpath)
    # print(model._tx_loaded_models)
    reg_models = mm._tx_model_repository.all_models.filename_to_model
    mimports = [val for key, val in reg_models.items() if val != model]
    return (model, mimports)


def get_grammar(debug=False):
    with open(join(GRAMMAR_DIR, 'rosbridge.tx')) as f:
        return f.read()