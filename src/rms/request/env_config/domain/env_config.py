
class EnvConfig():
    def __init__(
        self,
        header: dict = None,
        setInfo: dict = None
    ) -> None:
        self.header: dict = header
        self.setInfo: dict = setInfo
        

__all__ = ['env_config']