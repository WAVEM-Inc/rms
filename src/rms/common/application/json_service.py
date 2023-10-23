import json

class JsonEncoder(json.JSONEncoder):
    def default(self, obj):
        if hasattr(obj, '__dict__'):
            return {'__class__': obj.__class__.__name__, '__dict__': obj.__dict__}
        return super().default(obj)