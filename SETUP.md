# Setup

## Running code

Enter an interactive shell into a container:

```bash
docker-compose run --rm nav
```

Run a one off command

```bash
docker-compose run --rm nav <command>
```

## Testing

```bash
docker-compose run --rm nav pytest --cov=./ --cov-report=xml
```
