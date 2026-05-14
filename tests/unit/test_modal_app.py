def test_app_imports_cleanly():
    from wdt_modal import app

    assert app.app.name == "warehouse-digital-twin"
