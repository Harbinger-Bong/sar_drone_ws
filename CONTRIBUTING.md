# Contributing to SAR Drone Simulation

Thank you for your interest in contributing to the SAR Drone Simulation project! 🚁

## 🤝 How to Contribute

### Reporting Issues
- Use the [GitHub issue tracker](https://github.com/your-username/sar-drone-simulation/issues)
- Search existing issues before creating new ones
- Provide clear steps to reproduce bugs
- Include system information (OS, ROS version, etc.)

### Feature Requests
- Open an issue with the "enhancement" label
- Describe the use case and expected behavior
- Consider implementation approach

### Code Contributions

#### Development Environment
1. **Fork** the repository
2. **Clone** your fork:
   ```bash
   git clone https://github.com/YOUR-USERNAME/sar-drone-simulation.git
   cd sar-drone-simulation
   ```
3. **Setup** development environment:
   ```bash
   ./scripts/setup.sh
   ```
4. **Open** in VS Code:
   ```bash
   code sar_drone_ws.code-workspace
   ```

#### Making Changes
1. **Create** a feature branch:
   ```bash
   git checkout -b feature/amazing-new-feature
   ```
2. **Make** your changes following our coding standards
3. **Test** thoroughly:
   ```bash
   ./scripts/build.sh
   colcon test --packages-select sar_drone_description
   ```
4. **Commit** with descriptive messages:
   ```bash
   git commit -m "feat: add autonomous landing capability"
   ```

#### Pull Request Process
1. **Update** documentation for any API changes
2. **Add** tests for new functionality  
3. **Ensure** CI/CD pipeline passes
4. **Submit** pull request with:
   - Clear description of changes
   - Issue reference (if applicable)
   - Screenshots/videos for UI changes

## 📋 Coding Standards

### ROS 2 Guidelines
- Follow [ROS 2 style guide](https://docs.ros.org/en/foxy/Contributing/Code-Style-Language-Versions.html)
- Use descriptive node and topic names
- Include proper error handling
- Add parameter validation

### Python Code Style
- Follow [PEP 8](https://www.python.org/dev/peps/pep-0008/)
- Use type hints where appropriate
- Maximum line length: 100 characters
- Use meaningful variable names

### C++ Code Style  
- Follow [Google C++ Style Guide](https://google.github.io/styleguide/cppguide.html)
- Use `snake_case` for variables and functions
- Use `PascalCase` for classes
- Include proper documentation comments

### XML/URDF Style
- Proper indentation (2 spaces)
- Meaningful link and joint names
- Include comments for complex sections
- Validate XML syntax

### Launch File Style
- Use descriptive argument names
- Include default values
- Add documentation strings
- Group related arguments

## 🧪 Testing Requirements

### Unit Tests
- Add tests for new functions/classes
- Aim for >80% code coverage
- Use appropriate ROS 2 testing frameworks

### Integration Tests
- Test complete workflows
- Verify sensor data flow
- Check navigation behavior
- Validate launch files

### Manual Testing Checklist
- [ ] Gazebo launches without errors
- [ ] Robot spawns correctly
- [ ] All sensors publish data
- [ ] Navigation stack starts
- [ ] RViz displays properly
- [ ] Performance acceptable

## 📝 Documentation Guidelines

### Code Documentation
- Document all public APIs
- Include parameter descriptions
- Provide usage examples
- Keep comments up-to-date

### README Updates
- Update feature lists
- Add new configuration options
- Include troubleshooting steps
- Update installation requirements

### Wiki Contributions
- Create tutorials for new features
- Add architecture diagrams
- Include performance benchmarks
- Provide troubleshooting guides

## 🎯 Priority Areas

We especially welcome contributions in these areas:

### 🔴 High Priority
- Performance optimization
- Bug fixes and stability
- Hardware integration support
- Real-world testing validation

### 🟡 Medium Priority
- Additional sensor integration
- Advanced navigation algorithms
- Multi-drone coordination
- Mission planning improvements

### 🟢 Low Priority
- UI/UX enhancements
- Additional visualization tools
- Alternative simulation environments
- Educational materials

## 🏷️ Issue Labels

- `bug` - Something isn't working
- `enhancement` - New feature request
- `documentation` - Documentation improvements
- `good first issue` - Good for newcomers
- `help wanted` - Extra attention needed
- `priority:high` - Critical issues
- `ros2` - ROS 2 specific
- `gazebo` - Simulation related
- `navigation` - Nav2 stack related

## 🚀 Development Workflow

### Branch Naming
- `feature/` - New features
- `bugfix/` - Bug fixes  
- `hotfix/` - Critical fixes
- `docs/` - Documentation only
- `refactor/` - Code restructuring

### Commit Messages
Follow [Conventional Commits](https://www.conventionalcommits.org/):
- `feat:` - New feature
- `fix:` - Bug fix
- `docs:` - Documentation
- `style:` - Formatting
- `refactor:` - Code restructuring
- `test:` - Adding tests
- `chore:` - Maintenance

### Release Process
1. Version bumping in `package.xml`
2. Changelog updates
3. Tag creation: `v1.2.3`
4. GitHub release with notes

## 🔒 Security

### Reporting Vulnerabilities
- **DO NOT** create public issues for security vulnerabilities
- Email: security@your-domain.com
- Provide detailed reproduction steps
- Allow 90 days for fix before disclosure

### Security Considerations
- Validate all input parameters
- Sanitize file paths
- Check network communications
- Audit dependencies regularly

## 📞 Getting Help

### Communication Channels
- **GitHub Discussions** - General questions
- **Discord/Slack** - Real-time chat (if available)
- **Email** - Direct contact for complex issues

### Learning Resources
- [ROS 2 Tutorials](https://docs.ros.org/en/foxy/Tutorials.html)
- [Gazebo Documentation](http://gazebosim.org/tutorials)
- [Nav2 Documentation](https://navigation.ros.org/)

## 🏆 Recognition

Contributors will be:
- Listed in README acknowledgments
- Tagged in release notes
- Invited to maintainer discussions (for significant contributions)

Thank you for making SAR Drone Simulation better! 🙏