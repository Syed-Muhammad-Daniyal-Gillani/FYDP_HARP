## 🚀 Auto-Activate Virtual Environment for `ROS_FYDP`

This setup ensures that the virtual environment (`venv`) in `ROS_FYDP` **activates automatically** when entering the folder and deactivates when leaving.

### **🔹 Setup Instructions**

1. Open `.bashrc`:  
   ```bash
   nano ~/.bashrc
   ```  
2. Add this at the end:  
   ```bash
   # Auto-activate venv in ROS_FYDP
   if [[ "$PWD" == "$HOME/ROS_FYDP" ]]; then
       source "$HOME/ROS_FYDP/venv/bin/activate"
   fi

   cd() {
       builtin cd "$@"  
       if [[ "$PWD" == "$HOME/ROS_FYDP" ]]; then
           source "$HOME/ROS_FYDP/venv/bin/activate"
       elif [[ -n "$VIRTUAL_ENV" && "$PWD" != "$HOME/ROS_FYDP"* ]]; then
           deactivate
       fi
   }
   ```  
3. Save and apply changes:  
   ```bash
   source ~/.bashrc
   ```

---

### **🔹 Installing Dependencies**  
After activation, install required dependencies:  
```bash
pip install --upgrade pip setuptools wheel  
pip install -r ~/ROS_FYDP/requirements.txt  
```

Now, your virtual environment will **automatically activate when you enter `ROS_FYDP`** and deactivate when leaving! 🎯

